"""Single-process session state and coherent calculations, shared by GUI/headless."""
import copy
import csv
import io
import json
import math
import re
from pathlib import Path
import uuid
import numpy as np
import yaml
from .math3d import point, validate_transform, vector, inverse, transform
from .model import Model
from .reference import Registry


MODES = ('declared', 'reference', 'candidate')


def aggregate(rows, selected, mode):
    selected = sorted(set(selected))
    included, missing, reasons, constituents, record_ids = [], [], [], {}, set()
    for name in selected:
        entry = rows.get(name, {}).get(mode)
        if not entry or entry.get('cog_fixed_m') is None or not math.isfinite(entry['mass_kg']) or entry['mass_kg'] <= 0:
            missing.append(name)
            continue
        rid = entry.get('record_id', name)
        if rid in record_ids:
            reasons.append('Duplicate physical record: ' + rid)
        record_ids.add(rid)
        for member in entry.get('constituents', []):
            if member in constituents:
                reasons.append('Overlapping mass constituent %s: %s/%s' % (member, constituents[member], name))
            constituents[member] = name
        included.append(name)
    for name in included:
        for other in rows[name][mode].get('overlaps_with', []):
            if other in record_ids:
                reasons.append('Known overlapping record: ' + other)
    mass = sum(rows[n][mode]['mass_kg'] for n in included)
    cog = None if mass <= 0 or reasons else (sum((rows[n][mode]['mass_kg']*np.array(rows[n][mode]['cog_fixed_m']) for n in included), np.zeros(3))/mass).tolist()
    return {'status': 'complete' if selected and not missing and not reasons and cog is not None else 'incomplete',
            'mass_kg': mass if not missing and not reasons and included else None,
            'cog_fixed_m': cog if not missing and not reasons else None,
            'known_subset_mass_kg': mass if not reasons else None, 'known_subset_cog_fixed_m': cog,
            'included': included, 'missing': missing, 'diagnostics': reasons,
            'composition_status': 'unverified' if mode == 'declared' or any(rows[n][mode].get('composition_status') != 'verified' for n in included) else 'verified',
            'calibration_status': 'declared' if mode == 'declared' else ('verified' if mode == 'reference' and included and all(rows[n][mode].get('verified') for n in included) else 'assumptions remain')}


class Session:
    def __init__(self, model, registry, project_root, instance=None):
        self.project_root = Path(project_root).resolve()
        self.instance = instance or uuid.uuid4().hex[:10]
        if not re.fullmatch(r'[A-Za-z0-9_]+', self.instance):
            raise ValueError('Instance must contain only letters, digits and underscores')
        self.original_registry = copy.deepcopy(registry.data)
        self.registry = registry
        self.model, self.generation = model, 1
        self.members = set()
        self.visible = set(model.links)
        self.inspected = next(iter(model.links))
        self.root = model.root_of(self.inspected)
        self.joints, self.joint_origins, self.visual_poses = {}, {}, {}
        self.preview_root_pose, self.pose_note = transform(), 'Offline model FK; component root at identity'
        self.explosion = 0.0
        self.alpha, self.axis_length = 0.6, 0.06
        self.overlays = {'meshes': True, 'axes': True, 'labels': True, 'ghost': False, 'joints': False,
                         'declared': True, 'reference': False, 'candidate': False}
        self.pose_source, self.fixed_frame, self.tf_prefix = 'preview', 'cog_viz_'+self.instance+'/world', ''
        self.latency, self.max_pose_age = 0.1, 0.5
        self.threshold_m, self.last_error = 0.001, ''
        self.last_report = None

    def replace_model(self, model):
        # Construct/validate first; callers keep the old model on failure.
        members, visible = self.members & set(model.links), self.visible & set(model.links)
        self.model, self.generation = model, self.generation+1
        self.members, self.visible = members, visible
        self.inspected = self.inspected if self.inspected in model.links else next(iter(model.links))
        self.root = model.root_of(self.inspected)
        self.joints, self.joint_origins, self.visual_poses = {}, {}, {}
        self.preview_root_pose, self.pose_note = transform(), 'Offline model FK; component root at identity'
        self.last_report, self.last_error = None, ''

    def reset_overrides(self):
        self.joints, self.joint_origins, self.visual_poses = {}, {}, {}
        self.preview_root_pose, self.pose_note = transform(), 'Offline model FK; component root at identity'
        self.explosion = 0.0

    def add_chain(self, target, include_masses):
        path = self.model.chain(self.inspected, target)
        self.visible.update(path)
        self.members.update(path if include_masses else [target])
        return path

    def preview_poses(self):
        return {k: self.preview_root_pose @ v for k,v in self.model.fk(self.root, self.joints, self.joint_origins).items()}

    def copy_live_snapshot(self, poses, stamp):
        if self.pose_source != 'live':
            raise ValueError('Copy requires a live TF snapshot')
        model, values = self.model, {}
        required = {n for n in model.links if model.root_of(n) == self.root}
        if not required <= set(poses):
            raise ValueError('Cannot copy incomplete component snapshot: '+', '.join(sorted(required-set(poses))))
        for name,j in model.joints.items():
            if j.child not in required or j.kind == 'fixed':
                continue
            if j.kind not in ('revolute','continuous','prismatic'):
                raise ValueError('Cannot infer snapshot for unsupported joint '+name)
            relative = inverse(j.pose) @ inverse(poses[j.parent]) @ poses[j.child]
            if j.kind == 'prismatic':
                q = float(np.dot(relative[:3,3], j.axis))
            else:
                r = relative[:3,:3]
                sine = np.dot(j.axis, [r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])/2
                q = math.atan2(sine, (np.trace(r)-1)/2)
            if not j.mimic:
                values[name] = q
        local = model.fk(self.root, values)
        root_pose = poses[self.root]
        for name in required:
            delta = inverse(root_pose @ local[name]) @ poses[name]
            angle = math.acos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1))
            if np.linalg.norm(delta[:3,3]) > .001 or angle > .01:
                raise ValueError('TF/model disagreement prevents joint snapshot: '+name)
        self.joints, self.joint_origins, self.visual_poses = values, {}, {}
        self.preview_root_pose = root_pose.copy()
        self.pose_source, self.fixed_frame = 'preview', 'cog_viz_'+self.instance+'/world'
        self.pose_note = 'Copied live TF snapshot at %.6f; inferred joint positions, continuous turns ambiguous; private preview coordinates retain snapshot placement' % stamp

    def evaluate(self, poses, stamp, pose_diagnostics=()):
        model, rows = self.model, {}
        for name, link in model.links.items():
            warnings = list(link.diagnostics)
            t = poses.get(name)
            if t is not None:
                validate_transform(t)
            else:
                warnings.append('Pose unavailable in selected component/reference frame')
            row = {'link': name, 'variant': model.variant, 'selected_mass': name in self.members,
                   'geometry_visible': name in self.visible, 'diagnostics': warnings,
                   'link_pose_fixed': None if t is None else t.tolist(),
                   'inertial_pose_local': None if link.mass is None else link.inertial.tolist(),
                   'visuals': [{'index': i, 'name': v.name, 'mesh_uri': v.uri, 'scale': v.size.tolist(), 'sha256': v.mesh_hash,
                                'origin_fingerprint': v.fingerprint, 'model_pose_local': v.pose.tolist(),
                                'current_pose_local': self.visual_poses.get((name,i),v.pose).tolist()} for i,v in enumerate(link.visuals)],
                   'child_joints': [{'name': j.name, 'type': j.kind, 'child': j.child, 'axis_in_joint': j.axis.tolist(),
                                     'limits': [j.lower,j.upper] if j.kind in ('revolute','prismatic') else None,
                                     'continuous_ui_range': [j.lower,j.upper] if j.kind == 'continuous' else None,
                                     'origin_local': self.joint_origins.get(j.name,j.pose).tolist(), 'mimic': j.mimic}
                                    for j in (model.joints[k] for k in model.children.get(name, []))]}
            if link.mass is not None and math.isfinite(link.mass) and link.mass > 0:
                row['declared'] = {'mass_kg': link.mass, 'cog_local_m': link.inertial[:3, 3].tolist(), 'label': 'URDF declared',
                                   'tensor_at_cog_link_kg_m2': None if link.tensor is None else (link.inertial[:3,:3] @ link.tensor @ link.inertial[:3,:3].T).tolist()}
            else:
                warnings.append('Missing/invalid declared mass')
            ref = self.registry.resolve(model, name, self.visual_poses)
            warnings.extend(ref['diagnostics'])
            row['registration'] = {k: v for k,v in ref.items() if k not in ('record','reference','candidate','cad_pose','ghost_pose')}
            if 'cad_pose' in ref:
                row['registration']['cad_pose_local'] = ref['cad_pose'].tolist()
                row['registration']['ghost_mesh_pose_local'] = None if 'ghost_pose' not in ref else ref['ghost_pose'].tolist()
                row['registration']['trust'] = {k: (ref['record'].get(k) or {}).get('status','unknown') for k in ('source','cad_to_mesh','calibrated_cad_to_link')}
            for mode in ('reference','candidate'):
                if mode in ref:
                    row[mode] = ref[mode]
            for mode in MODES:
                if mode in row:
                    row[mode]['cog_fixed_m'] = None if t is None else point(t, row[mode]['cog_local_m']).tolist()
            for other in ('reference', 'candidate'):
                if 'declared' in row and other in row:
                    delta = (np.array(row['declared']['cog_local_m'])-row[other]['cog_local_m'])*1000
                    row[other]['declared_minus_source_mm'] = delta.tolist()
                    row[other]['discrepancy_mm'] = float(np.linalg.norm(delta))
                    row[other]['above_threshold'] = bool(np.linalg.norm(delta) > self.threshold_m*1000)
                    row[other]['mass_difference_kg'] = row['declared']['mass_kg']-row[other]['mass_kg']
            rows[name] = row
        declared_links = {n for n,l in model.links.items() if l.mass is not None}
        report = {'schema_version': 1, 'instance': self.instance, 'generation': self.generation,
                  'model_source': model.source, 'model_sha256': model.fingerprint, 'variant': model.variant,
                  'pose_source': self.pose_source, 'frame': self.fixed_frame, 'timestamp': float(stamp),
                  'root_anchor': self.root if self.pose_source == 'preview' else None,
                  'pose_note': self.pose_note if self.pose_source == 'preview' else 'External TF only; no model FK applied',
                  'diagnostics': list(model.diagnostics)+list(pose_diagnostics)+([self.last_error] if self.last_error else []),
                  'rows': rows, 'selected': sorted(self.members), 'excluded': sorted(set(rows)-self.members),
                  'aggregates': {mode: aggregate(rows, self.members, mode) for mode in MODES},
                  'all_declared_urdf': aggregate(rows, declared_links, 'declared'),
                  'all_declared_excluded_no_inertial': sorted(set(rows)-declared_links),
                  'membership_difference_from_all_declared': sorted(declared_links ^ self.members),
                  'enabled_overlays': dict(self.overlays), 'discrepancy_threshold_mm': self.threshold_m*1000}
        self.last_report = report
        return report

    def export_folder(self):
        intended = self.project_root / 'exports' / 'cog_viewer' / self.instance
        folder = intended.resolve()
        if self.project_root not in folder.parents or folder != intended:
            raise ValueError('Export folder must be inside project')
        folder.mkdir(parents=True, exist_ok=True)
        return folder

    def write_export(self, name, text):
        folder = self.export_folder()
        path = (folder / name).resolve()
        if path.parent != folder:
            raise ValueError('Exports must remain inside viewer folder')
        path.write_text(text)
        return path

    def session_data(self):
        return {'schema_version': 1, 'model_xml': self.model.xml, 'model_source': self.model.source,
                'variant': self.model.variant, 'members': sorted(self.members), 'visible': sorted(self.visible),
                'inspected': self.inspected, 'root': self.root, 'joints': self.joints,
                'joint_origins': {k:v.tolist() for k,v in self.joint_origins.items()},
                'visual_poses': [{'link': k[0], 'index': k[1], 'matrix': v.tolist()} for k,v in self.visual_poses.items()],
                'registry': self.registry.data, 'overlays': self.overlays, 'explosion': self.explosion,
                'pose_source_at_save': self.pose_source, 'fixed_frame_at_save': self.fixed_frame,
                'tf_prefix': self.tf_prefix, 'preview_root_pose': self.preview_root_pose.tolist(), 'pose_note': self.pose_note,
                'alpha': self.alpha, 'axis_length': self.axis_length, 'threshold_m': self.threshold_m,
                'original_registry': self.original_registry}

    def restore(self, data):
        if data.get('schema_version') != 1:
            raise ValueError('Unsupported session schema')
        model = Model(data['model_xml'], self.project_root, data['variant'], data['model_source'])
        registry = Registry(data['registry'])
        original = Registry(data.get('original_registry',data['registry']))
        members, visible = set(data['members']), set(data['visible'])
        if not (members | visible) <= set(model.links) or data['inspected'] not in model.links:
            raise ValueError('Session refers to missing links')
        origins = {k:validate_transform(v) for k,v in data['joint_origins'].items()}
        vp = {(v['link'], int(v['index'])):validate_transform(v['matrix']) for v in data['visual_poses']}
        for (name, idx) in vp:
            if name not in model.links or not 0 <= idx < len(model.links[name].visuals):
                raise ValueError('Invalid session visual binding')
        if not set(origins) <= set(model.joints) or not set(data['joints']) <= set(model.joints):
            raise ValueError('Invalid session joint binding')
        model.fk(data['root'], data['joints'], origins)
        for name in model.joints:
            model.joint_value(name, data['joints'])
        root_pose = validate_transform(data.get('preview_root_pose', np.eye(4)))
        explosion = float(data.get('explosion', 0))
        if not math.isfinite(explosion) or explosion < 0:
            raise ValueError('Invalid explosion')
        alpha, axis_length, threshold = (float(data.get(k,d)) for k,d in [('alpha',.6),('axis_length',.06),('threshold_m',.001)])
        if not np.isfinite([alpha,axis_length,threshold]).all() or not 0 <= alpha <= 1 or axis_length < 0 or threshold < 0:
            raise ValueError('Invalid display or threshold settings')
        overlays = data.get('overlays', {})
        if not isinstance(overlays,dict) or any(not isinstance(v,bool) for v in overlays.values()):
            raise ValueError('Overlay flags must be boolean values')
        self.replace_model(model)
        self.registry, self.members, self.visible = registry, members, visible
        self.original_registry = original.data
        self.inspected, self.root, self.joints = data['inspected'], data['root'], data['joints']
        self.joint_origins, self.visual_poses, self.explosion = origins, vp, explosion
        self.preview_root_pose = root_pose
        self.pose_note = data.get('pose_note', 'Restored offline model preview')
        self.alpha, self.axis_length, self.threshold_m = alpha, axis_length, threshold
        self.tf_prefix = str(data.get('tf_prefix','')).strip('/')
        self.overlays.update({k:v for k,v in overlays.items() if k in self.overlays})
        # Restored sessions always start as isolated previews, never reattach to a live robot implicitly.
        self.pose_source, self.fixed_frame = 'preview', 'cog_viz_'+self.instance+'/world'

    def export(self):
        if self.last_report is None:
            raise ValueError('No current report to export')
        report = self.last_report
        self.write_export('session.yaml', yaml.safe_dump(self.session_data(), sort_keys=False))
        self.write_export('reference.yaml', yaml.safe_dump(self.registry.data, sort_keys=False))
        self.write_export('report.json', json.dumps(report, indent=2, allow_nan=False))
        output = io.StringIO()
        fields = ['link','variant','source','mass_kg','cog_local_m','cog_fixed_m','label','diagnostics']
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        snippets = ['<!-- CoG-origin suggestions ONLY; no tensor/mass block. Review CAD placement and axes before applying. -->']
        for name, row in report['rows'].items():
            for mode in MODES:
                if mode not in row:
                    continue
                e = row[mode]
                writer.writerow(dict(link=name, variant=self.model.variant, source=mode, mass_kg=e['mass_kg'],
                                     cog_local_m=e['cog_local_m'], cog_fixed_m=e['cog_fixed_m'], label=e['label'], diagnostics='; '.join(row['diagnostics'])))
                if mode == 'candidate' and name in self.members:
                    # A partial origin suggestion deliberately omits RPY/tensor: this is not a complete inertial block.
                    provenance = json.dumps({'variant': self.model.variant, 'link': name, 'original_xyz': row.get('declared', {}).get('cog_local_m'), 'source': e.get('provenance')}).replace('--', '−−')
                    snippets.extend(['<!-- '+provenance+' -->', '<origin xyz="%s" />' % ' '.join('%.12g' % x for x in e['cog_local_m'])])
        self.write_export('report.csv', output.getvalue())
        self.write_export('candidate_origins.xml', '\n'.join(snippets)+'\n')
        return self.export_folder()
