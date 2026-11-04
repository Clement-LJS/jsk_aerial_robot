"""Marker construction and owned-marker lifecycle. Only this layer imports messages."""
import copy
import functools
import threading
import numpy as np
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from .math3d import point, quaternion, transform
from .reference import placement


COLORS = {'declared': (1, .12, .12), 'reference': (.15, 1, .25), 'candidate': (1, .65, .08)}


def synchronized(method):
    @functools.wraps(method)
    def call(self, *args, **kwargs):
        with self.lock:
            return method(self, *args, **kwargs)
    return call


def tf_name(prefix, link):
    prefix, link = prefix.strip('/'), link.strip('/')
    return link if not prefix or link == prefix or link.startswith(prefix+'/') else prefix+'/'+link


class Scene:
    def __init__(self, publisher, instance):
        self.publisher, self.instance, self.previous = publisher, instance, {}
        self.lock = threading.RLock()
        self.markers = []

    def marker(self, body, overlay, kind, pose=None, color=(1,1,1), alpha=1, size=(.02,.02,.02), frame=None, locked=False):
        m = Marker()
        m.header.frame_id = frame or self.frame
        m.header.stamp = self.stamp
        m.ns, m.id = 'cog_viz/'+self.instance+'/'+body+'/'+overlay, 0
        m.type, m.action, m.frame_locked = kind, Marker.ADD, locked
        pose = transform() if pose is None else pose
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pose[:3, 3]
        q = quaternion(pose)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        m.scale.x, m.scale.y, m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = (*color, alpha)
        self.markers.append(m)
        return m

    def label(self, body, key, text, xyz, size=.025, **kwargs):
        m = self.marker(body, key, Marker.TEXT_VIEW_FACING, transform(xyz), size=(0,0,size), **kwargs)
        m.text = text

    def line(self, body, key, points, color=(1,1,1), width=.003, **kwargs):
        m = self.marker(body, key, Marker.LINE_STRIP, color=color, size=(width,0,0), **kwargs)
        m.points = [Point(*p) for p in points]

    def axes(self, body, key, pose, length, labels, **kwargs):
        for i, color in enumerate(((1,0,0),(0,1,0),(0,.4,1))):
            m = self.marker(body, key+'/'+str(i), Marker.ARROW, color=color, size=(.003,.007,.012), **kwargs)
            end = pose[:3, 3]+pose[:3, i]*length
            m.points = [Point(*pose[:3, 3]), Point(*end)]
        if labels:
            caption, offset = key, [0,0,.02]
            if key.startswith('link '):
                caption, offset = 'Link origin', [-.08,-.06,.06]
            elif key.startswith('mesh '):
                caption, offset = 'Mesh origin '+key.split(' ',1)[1], [.06,.06,.06]
            elif key == 'inertial (URDF)':
                caption, offset = 'Inertial / CoG axes', [.08,-.06,-.05]
            elif key == 'CAD (saved)':
                caption, offset = 'CAD origin (saved)', [0,-.09,.08]
            label_pos = pose[:3,3]+offset
            self.label(body, key+'/name', caption, label_pos, size=.014, **kwargs)
            self.line(body,key+'/leader',[pose[:3,3],label_pos],color=(.7,.7,.7),width=.001,**kwargs)

    def geometry(self, body, key, visual, pose, alpha):
        kind = {'mesh': Marker.MESH_RESOURCE, 'box': Marker.CUBE, 'sphere': Marker.SPHERE, 'cylinder': Marker.CYLINDER}[visual.kind]
        m = self.marker(body, key, kind, pose, color=(.65,.75,.85), alpha=alpha, size=visual.size)
        if visual.kind == 'mesh':
            m.mesh_resource, m.mesh_use_embedded_materials = visual.uri, False

    @synchronized
    def render(self, session, poses, report):
        self.frame, self.stamp, self.markers = session.fixed_frame, rospy.Time.from_sec(report['timestamp']), []
        overlays, length = session.overlays, session.axis_length
        for ordinal, name in enumerate(sorted(session.visible)):
            if name not in poses:
                continue
            link = session.model.links[name]
            # Explosion changes only drawing transforms. Evaluation received the unmodified poses.
            display = transform([ordinal*session.explosion, 0, 0]) @ poses[name]
            row = report['rows'][name]
            if overlays['axes']:
                self.axes(name, 'link '+name, display, length, overlays['labels'])
                if link.mass is not None:
                    self.axes(name, 'inertial (URDF)', display @ link.inertial, length*.8, overlays['labels'])
            for i, visual in enumerate(link.visuals):
                vm = session.visual_poses.get((name, i), visual.pose)
                if overlays['meshes']:
                    self.geometry(name, 'mesh/'+str(i), visual, display @ vm, session.alpha)
                if overlays['axes']:
                    self.axes(name, 'mesh '+str(i), display @ vm, length, overlays['labels'])
            ref = session.registry.resolve(session.model, name, session.visual_poses)
            if 'cad_pose' in ref:
                if overlays['axes'] and (overlays['reference'] or overlays['candidate'] or overlays['ghost']):
                    self.axes(name, 'CAD (saved)', display @ ref['cad_pose'], length, overlays['labels'])
                    source = ref['record']['source']
                    if source.get('inertia_at_cog_kg_m2') is not None:
                        tensor_pose = ref['cad_pose'] @ transform(rpy=source.get('inertia_axes_rpy_in_cad',[0,0,0]))
                        tensor_pose[:3,3] = ref['reference']['cog_local_m']
                        self.axes(name, 'inertial (reference)', display @ tensor_pose, length*.8, overlays['labels'])
                if overlays['ghost'] and 'ghost_pose' in ref:
                    self.geometry(name, 'reference_ghost', link.visuals[ref['visual_index']], display @ ref['ghost_pose'], .18)
            for mode, color in COLORS.items():
                entry = row.get(mode)
                if not overlays[mode] or entry is None:
                    continue
                if mode == 'reference' and not entry['verified']:
                    color = COLORS['candidate']
                pos = point(display, entry['cog_local_m'])
                self.marker(name, mode+'/cog', Marker.SPHERE, transform(pos), color=color, size=(.01,)*3)
                if overlays['labels']:
                    offsets = {'declared':[0,.12,.03],'reference':[0,-.12,.03],'candidate':[0,0,.15]}
                    label_pos = pos+offsets[mode]
                    single_total = report['aggregates'][mode]['included']==[name] and report['aggregates'][mode]['status']=='complete'
                    text = '%s\n%s\n%.6f kg%s' % (name, entry['label'], entry['mass_kg'], ' (selected total)' if single_total else '')
                    self.label(name, mode+'/label', text, label_pos, size=.014, color=color)
                    self.line(name,mode+'/label_leader',[pos,label_pos],color=color,width=.001)
                if mode != 'declared' and 'declared' in row and overlays['declared']:
                    self.line(name, mode+'/delta', [point(display, row['declared']['cog_local_m']), pos], color=color)
                    if overlays['labels']:
                        self.label(name, mode+'/error', 'Δ %.3f mm' % entry['discrepancy_mm'], pos+[0,0,-.025], size=.018, color=color)
            if overlays['joints']:
                for joint_name in session.model.children.get(name, []):
                    j = session.model.joints[joint_name]
                    jt = display @ session.joint_origins.get(joint_name, j.pose)
                    self.marker(name, 'joint/'+joint_name, Marker.SPHERE, jt, color=(.8,.3,1), size=(.012,)*3)
                    if j.kind != 'fixed' and np.linalg.norm(j.axis) > 0:
                        m = self.marker(name, 'joint/'+joint_name+'/axis', Marker.ARROW, color=(.8,.3,1), size=(.004,.01,.015))
                        m.points = [Point(*jt[:3,3]), Point(*point(jt, j.axis*length*1.5))]
                    if overlays['labels']:
                        self.label(name, 'joint/'+joint_name+'/label', joint_name+' ('+j.kind+')', jt[:3,3]+[0,0,-.02], size=.018)
        for mode, agg in report['aggregates'].items():
            if not overlays[mode] or agg['cog_fixed_m'] is None:
                continue
            color = COLORS['candidate'] if mode == 'reference' and agg['calibration_status'] != 'verified' else COLORS[mode]
            pos = np.array(agg['cog_fixed_m'])
            single = len(agg['included'])==1
            self.marker('aggregate', mode, Marker.SPHERE, transform(pos), color=color, size=(.016 if single else .03,)*3)
            if not single:
                self.label('aggregate', mode+'/label', '%s combined %.6f kg\n%s' % (mode, agg['mass_kg'], agg['calibration_status']), pos+[0,0,.12+.04*list(COLORS).index(mode)], color=color)
        aggregate_status = '\n'.join('%s: %s%s' % (mode, agg['status'], ' %.6f kg' % agg['mass_kg'] if agg['mass_kg'] is not None else ' | missing: '+', '.join(agg['missing'][:4])+'; see status for details') for mode,agg in report['aggregates'].items() if overlays[mode])
        legend = ('%s | %s | generation %d\n%s | %s | t=%.6f\nCoG overlays: %s\nRed: URDF; green: verified CAD; amber: assumptions/candidate\nSelected mass bodies: %s\n%s\n%s' %
                  (session.instance, session.model.variant, session.generation, session.pose_source, session.fixed_frame, report['timestamp'], ', '.join(k for k in COLORS if overlays[k]), ', '.join(sorted(session.members)),
                   aggregate_status,
                   '\n'.join(report['diagnostics'][:5])+ ('\nMore diagnostics in status/window' if len(report['diagnostics']) > 5 else '')))
        self.label('legend', 'status', legend, [-.4,0,.65], size=.025)
        self.publish()

    @synchronized
    def legacy(self, model, prefix, only_links):
        self.frame, self.stamp, self.markers = '', rospy.Time(0), []
        for name, link in model.links.items():
            if link.mass is None or (only_links and name not in only_links):
                continue
            kw = {'frame': tf_name(prefix, name), 'locked': True}
            self.marker(name, 'declared/cog', Marker.SPHERE, transform(link.inertial[:3,3]), color=COLORS['declared'], size=(.025,)*3, **kw)
            self.axes(name, 'inertial (URDF)', link.inertial, .06, False, **kw)
            self.label(name, 'declared/text', '%s\nmass %.6f kg\nCoG %s m\nI_C (inertial axes) %s' % (name, link.mass, link.inertial[:3,3].tolist(), None if link.tensor is None else link.tensor.tolist()), link.inertial[:3,3]+[0,0,.06], **kw)
        self.publish()

    @synchronized
    def publish(self):
        current = {(m.ns,m.id):m for m in self.markers}
        deleted = []
        for key in self.previous.keys()-current.keys():
            m = copy.deepcopy(self.previous[key])
            m.action = Marker.DELETE
            deleted.append(m)
        self.publisher.publish(MarkerArray(self.markers+deleted))
        self.previous = current

    @synchronized
    def clear(self):
        self.markers = []
        self.publish()
