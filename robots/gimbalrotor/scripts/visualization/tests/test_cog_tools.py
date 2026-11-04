"""Run from any cwd: python3 -B <project>/scripts/visualization/tests/test_cog_tools.py.

No node initialization, ROS master, GUI, build, controller, or hardware is used.
Optional Xacro/message-construction checks skip if their dependencies are absent.
"""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts/visualization'))
from cog_tools.math3d import (transform, point, motion, inverse, from_quaternion,
                              quaternion, rotate_inertia, import_properties, validate_transform)
from cog_tools.model import Model, load_model
from cog_tools.reference import Registry
from cog_tools.session import Session, aggregate


def inertial(mass=1, xyz='0 0 0', rpy='0 0 0'):
    return '<inertial><mass value="%s"/><origin xyz="%s" rpy="%s"/><inertia ixx="1" iyy="2" izz="3" ixy="0" ixz="0" iyz="0"/></inertial>' % (mass, xyz, rpy)


def simple_xml():
    return ('<robot name="fixture"><link name="root"/><link name="roll_joint_asm_link">'+inertial(.28288, '.02057 0 -.03110')+
            '<visual><origin xyz=".027 0 0"/><geometry><mesh filename="package://gimbalrotor/urdf/beetle/mesh/roll_joint_asm_roll_yaw.STL" scale=".001 .001 .001"/></geometry></visual></link>'+
            '<link name="roll_hand_link">'+inertial(.30806, '.17702 -.02352 .00828')+
            '<visual><geometry><mesh filename="package://gimbalrotor/urdf/beetle/mesh/perching_hand_roll_asm.STL" scale=".001 .001 .001"/></geometry></visual></link>'+
            '<joint name="mount" type="fixed"><parent link="root"/><child link="roll_joint_asm_link"/></joint>'+
            '<joint name="joint_roll" type="revolute"><parent link="roll_joint_asm_link"/><child link="roll_hand_link"/><origin xyz=".008 0 -.068"/><axis xyz="1 0 0"/><limit lower="-1.5707963267948966" upper="1.5707963267948966"/></joint></robot>')


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.registry = Registry.read(ROOT/'config/visualization/beetle_mass_reference.yaml')
        self.model = Model(simple_xml(), ROOT, 'roll_yaw', 'fixture')
        self.s = Session(self.model, self.registry, ROOT)
        self.s.members = {'roll_joint_asm_link', 'roll_hand_link'}

    def assertVec(self, actual, expected):
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)

    def test_translation_once_and_corrected_urdf(self):
        for _ in range(100):
            ref = self.registry.resolve(self.model, 'roll_joint_asm_link')
            self.assertVec(ref['reference']['cog_local_m'], [.04757,0,-.03110])
        corrected = Model(simple_xml().replace('.02057 0 -.03110', '.04757 0 -.03110'), ROOT, 'roll_yaw')
        self.s.replace_model(corrected)
        report = self.s.evaluate(self.s.preview_poses(), 1)
        self.assertAlmostEqual(report['rows']['roll_joint_asm_link']['reference']['discrepancy_mm'], 0)

    def test_rotation_order_and_quaternion(self):
        t = transform([.02,0,0], [0,0,math.pi/2])
        self.assertVec(point(t, [.1,0,0]), [.02,.1,0])
        for rpy in ([0,0,math.pi], [1,2,3], [.1,-.4,.2]):
            t = transform([1,2,3], rpy)
            self.assertVec(from_quaternion(t[:3,3], quaternion(t)).ravel(), t.ravel())
        with self.assertRaises(ValueError):
            from_quaternion([0,0,0], [0,0,0,2])
        with self.assertRaises(ValueError):
            validate_transform(np.zeros((4,4)))

    def test_joint_transform_once_nonidentity_ancestor_and_prismatic(self):
        t = transform([1,0,0], [0,0,math.pi/2])
        self.assertVec(point(t,[.1,0,0]), [1,.1,0])
        xml = ('<robot name="joint"><link name="a"/><link name="b"/><link name="c"/><link name="d"/>'
               '<joint name="ancestor" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 2 3" rpy="0 .3 .4"/></joint>'
               '<joint name="turn" type="revolute"><parent link="b"/><child link="c"/><origin xyz=".1 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/><limit lower="-2" upper="2"/></joint>'
               '<joint name="slide" type="prismatic"><parent link="c"/><child link="d"/><origin xyz="0 .2 0" rpy=".2 0 0"/><axis xyz="0 0 2"/><limit lower="0" upper="1"/></joint></robot>')
        m = Model(xml, ROOT)
        poses = m.fk('a', {'turn': .7,'slide': .25})
        ancestor = transform([1,2,3],[0,.3,.4])
        expected = ancestor @ transform([.1,0,0],[0,0,math.pi/2]) @ motion('revolute',[1,0,0],.7) @ transform([0,.2,0],[.2,0,0]) @ transform([0,0,.25])
        self.assertVec(poses['d'].ravel(), expected.ravel())

    def test_weighted_and_robot_roll_hand(self):
        rows = {'a': {'declared': {'mass_kg':1,'cog_fixed_m':[0,0,0]}}, 'b': {'declared': {'mass_kg':3,'cog_fixed_m':[1,0,0]}}}
        a = aggregate(rows, ['a','b','a'], 'declared')
        self.assertVec(a['cog_fixed_m'], [.75,0,0])
        self.assertEqual(a['mass_kg'],4)
        for angle, expected in [(0,[.119223377669476,-.0122610945273632,-.0460197502284496]),
                                (math.pi/2,[.119223377669476,-.00431640572646969,-.0625972504822825])]:
            self.s.joints['joint_roll'] = angle
            report = self.s.evaluate(self.s.preview_poses(),1)
            self.assertAlmostEqual(report['aggregates']['reference']['mass_kg'],.59094)
            self.assertVec(report['aggregates']['reference']['cog_fixed_m'],expected)

    def test_visual_only_vs_calibration_joint_and_explosion(self):
        initial = self.s.evaluate(self.s.preview_poses(),1)
        self.s.visual_poses[('roll_joint_asm_link',0)] = transform([.037,0,0])
        shifted = self.s.evaluate(self.s.preview_poses(),2)
        for mode in ('declared','reference'):
            self.assertVec(initial['rows']['roll_joint_asm_link'][mode]['cog_local_m'], shifted['rows']['roll_joint_asm_link'][mode]['cog_local_m'])
        self.assertAlmostEqual(shifted['rows']['roll_joint_asm_link']['registration']['mesh_placement_delta_m'], .01)
        self.assertVec(shifted['rows']['roll_joint_asm_link']['candidate']['cog_local_m'],[.05757,0,-.0311])
        before = copy.deepcopy(shifted['aggregates'])
        self.s.explosion, self.s.visible = 1, set()
        self.assertEqual(before,self.s.evaluate(self.s.preview_poses(),2)['aggregates'])
        self.s.joint_origins['mount'] = transform([1,2,3])
        moved = self.s.evaluate(self.s.preview_poses(),3)
        for mode in ('declared','reference','candidate'):
            self.assertVec(np.array(moved['aggregates'][mode]['cog_fixed_m'])-before[mode]['cog_fixed_m'], [1,2,3])
        self.assertVec(self.registry.resolve(self.model,'roll_joint_asm_link')['ghost_pose'][:3,3],[.027,0,0])

    def test_units_inertia_and_origin_rpy(self):
        m,c,i = import_properties(1000,[20.57,0,0],np.diag([1e9,2e9,3e9]),'g','mm','g_mm2')
        self.assertEqual(m,1)
        self.assertVec(c,[.02057,0,0])
        self.assertVec(rotate_inertia(i,transform([3,2,1],[0,0,math.pi/2])).ravel(), np.diag([2,1,3]).ravel())
        xml = '<robot name="r"><link name="a">'+inertial(1,'.1 0 0','0 0 1.5707963267948966')+'</link></robot>'
        s = Session(Model(xml,ROOT),Registry(),ROOT)
        e = s.evaluate(s.preview_poses(),1)['rows']['a']['declared']
        self.assertVec(e['cog_local_m'],[.1,0,0])
        self.assertVec(np.diag(e['tensor_at_cog_link_kg_m2']),[2,1,3])

    def test_membership_incomplete_invalid_and_overlap(self):
        r = self.s.evaluate(self.s.preview_poses(),1)
        self.s.members.remove('roll_hand_link')
        self.assertAlmostEqual(self.s.evaluate(self.s.preview_poses(),2)['aggregates']['reference']['mass_kg'],.28288)
        missing = self.s.evaluate({'roll_joint_asm_link': transform()},3)
        self.s.members.add('roll_hand_link')
        missing = self.s.evaluate({'roll_joint_asm_link': transform()},3)
        a = missing['aggregates']['declared']
        self.assertEqual(a['status'],'incomplete')
        self.assertIsNone(a['mass_kg'])
        self.assertIsNone(a['cog_fixed_m'])
        self.assertEqual(a['missing'],['roll_hand_link'])
        for mass in (0,-1,float('nan'),float('inf')):
            a = aggregate({'a':{'declared':{'mass_kg':mass,'cog_fixed_m':[0,0,0]}}},['a'],'declared')
            self.assertIsNone(a['cog_fixed_m'])
        rows = copy.deepcopy(r['rows'])
        rows['roll_hand_link']['reference']['constituents'] = ['roll_yaw_roll_assembly']
        self.assertIsNone(aggregate(rows,self.s.members,'reference')['cog_fixed_m'])
        with self.assertRaises(ValueError):
            import_properties(-1,[0,0,0])

    def test_variants_scale_mesh_and_multiple_visuals(self):
        m = Model(simple_xml(),ROOT,'pitch_roll')
        self.assertNotIn('reference',self.registry.resolve(m,'roll_joint_asm_link'))
        for replacement in ('.002 .001 .001','-.001 .001 .001'):
            m = Model(simple_xml().replace('.001 .001 .001',replacement),ROOT,'roll_yaw')
            self.assertNotIn('reference',self.registry.resolve(m,'roll_joint_asm_link'))
        doc = ET.fromstring(simple_xml())
        el = doc.find("link[@name='roll_joint_asm_link']")
        el.append(copy.deepcopy(el.find('visual')))
        m = Model(ET.tostring(doc,encoding='unicode'),ROOT,'roll_yaw')
        result = self.registry.resolve(m,'roll_joint_asm_link')
        self.assertIn('reference',result)
        self.assertNotIn('candidate',result)
        self.assertEqual(len(self.s.evaluate(self.s.preview_poses(),1)['aggregates']['declared']['included']),2)

    def test_disk_visual_reload_preserves_baseline_and_direct_cad_calibration(self):
        saved = copy.deepcopy(self.registry.data)
        model = Model(simple_xml().replace('xyz=".027 0 0"','xyz=".037 0 0"'),ROOT,'roll_yaw')
        self.s.replace_model(model)
        r = self.s.evaluate(self.s.preview_poses(),2)['rows']['roll_joint_asm_link']
        self.assertVec(r['reference']['cog_local_m'],[.04757,0,-.0311])
        self.assertVec(r['candidate']['cog_local_m'],[.05757,0,-.0311])
        self.assertEqual(saved,self.s.registry.data)
        data = copy.deepcopy(saved)
        record = data['records']['beetle_roll_yaw_roll_assembly']
        record['cad_to_mesh'] = None
        record['source']['status'] = 'verified'
        record['calibrated_cad_to_link']['status'] = 'verified'
        registry = Registry(data)
        resolved = registry.resolve(model,'roll_joint_asm_link')
        self.assertTrue(resolved['reference']['verified'])
        self.assertNotIn('candidate',resolved)
        self.assertNotIn('ghost_pose',resolved)
        with self.assertRaises(ValueError):
            data['units']['length'] = 'mm'
            Registry(data)

    def test_mimic_cycles_limits_and_disconnected(self):
        xml = ('<robot name="m"><link name="a"/><link name="b"/><link name="c"/>'
               '<joint name="j" type="continuous"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/></joint>'
               '<joint name="k" type="continuous"><parent link="b"/><child link="c"/><axis xyz="0 1 0"/><mimic joint="j" multiplier="2" offset=".1"/></joint></robot>')
        model = Model(xml,ROOT)
        self.assertAlmostEqual(model.joint_value('k',{'j':.3}),.7)
        with self.assertRaises(ValueError):
            Model(xml.replace('<axis xyz="0 0 1"/>','<mimic joint="k"/>'),ROOT)
        with self.assertRaises(ValueError):
            Model(xml.replace('0 0 1','0 0 0'),ROOT)
        with self.assertRaises(ValueError):
            self.model.fk('root',{'joint_roll':2})
        disconnected = Model('<robot name="r"><link name="a"/><link name="b"/></robot>',ROOT)
        self.assertEqual(set(disconnected.fk('a')),{'a'})
        with self.assertRaises(ValueError):
            disconnected.chain('a','b')
        unsupported = Model(xml.replace('type="continuous"','type="floating"',1),ROOT)
        self.assertEqual(set(unsupported.fk('a')),{'a'})

    def test_invalid_reload_transaction_and_session_roundtrip(self):
        previous = self.s.model
        with self.assertRaises(Exception):
            self.s.replace_model(Model('<bad>',ROOT))
        self.assertIs(self.s.model,previous)
        data = self.s.session_data()
        data['joint_origins'] = {'mount': np.zeros((4,4)).tolist()}
        with self.assertRaises(ValueError):
            self.s.restore(data)
        self.assertIs(self.s.model,previous)
        data = self.s.session_data()
        self.s.restore(data)
        self.assertEqual(self.s.pose_source,'preview')
        self.assertEqual(self.s.registry.data,self.registry.data)
        # Test real export files, contained inside the project, then clean this test's own files.
        with tempfile.TemporaryDirectory(prefix='cog_check_',dir=ROOT/'scripts/visualization/tests') as folder:
            s = Session(self.model,self.registry,Path(folder),'fixture')
            s.members = self.s.members
            s.evaluate(s.preview_poses(),1)
            dest = s.export()
            self.assertEqual(json.loads((dest/'report.json').read_text())['aggregates']['reference']['status'],'complete')
            self.assertNotIn('<inertia ',(dest/'candidate_origins.xml').read_text())
            self.assertEqual(yaml.safe_load((dest/'session.yaml').read_text())['variant'],'roll_yaw')
            with self.assertRaises(ValueError):
                s.write_export('../escape.txt','x')

    def test_copy_live_snapshot_to_preview_and_model_disagreement(self):
        self.s.joints = {'joint_roll':.6}
        self.s.preview_root_pose = transform([1,2,3],[.2,.1,.3])
        live = self.s.preview_poses()
        self.s.joints = {}
        self.s.pose_source = 'live'
        self.s.copy_live_snapshot(live,12.3)
        self.assertEqual(self.s.pose_source,'preview')
        self.assertAlmostEqual(self.s.joints['joint_roll'],.6)
        self.assertIn('12.300000', self.s.pose_note)
        for name in live:
            self.assertVec(self.s.preview_poses()[name].ravel(),live[name].ravel())
        self.s.pose_source = 'live'
        live['roll_hand_link'] = transform([.1,0,0]) @ live['roll_hand_link']
        with self.assertRaises(ValueError):
            self.s.copy_live_snapshot(live,13)
        self.assertEqual(self.s.pose_source,'live')

    def test_same_mesh_distinct_bodies_and_missing_origin_policy(self):
        uri = 'package://gimbalrotor/urdf/beetle/mesh/perching_hand_roll_asm.STL'
        visual = '<visual><geometry><mesh filename="'+uri+'"/></geometry></visual>'
        xml = ('<robot name="copies"><link name="a">'+inertial()+visual+'</link><link name="b">'+inertial(3)+visual+'</link>'
               '<joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 0 0"/></joint></robot>')
        s = Session(Model(xml,ROOT), Registry(), ROOT)
        s.members = {'a','b'}
        agg = s.evaluate(s.preview_poses(),1)['aggregates']['declared']
        self.assertEqual(agg['mass_kg'],4)
        self.assertVec(agg['cog_fixed_m'],[.75,0,0])
        model = Model('<robot name="origin"><link name="a">'+inertial().replace('<origin xyz="0 0 0" rpy="0 0 0"/>','')+'</link><link name="empty"/></robot>',ROOT)
        s = Session(model,Registry(),ROOT)
        r = s.evaluate(s.preview_poses(),1)
        self.assertVec(r['rows']['a']['declared']['cog_local_m'],[0,0,0])
        self.assertNotIn('declared',r['rows']['empty'])
        self.assertEqual(r['all_declared_urdf']['status'],'complete')
        self.assertEqual(r['all_declared_excluded_no_inertial'],['empty'])


@unittest.skipUnless(importlib.util.find_spec('xacro') and importlib.util.find_spec('rospkg'), 'Xacro/rospkg unavailable')
class CurrentModelTests(unittest.TestCase):
    def test_all_local_presets_and_grounded_roll_values(self):
        presets = yaml.safe_load((ROOT/'config/visualization/beetle_viewer_presets.yaml').read_text())
        registry = Registry.read(ROOT/'config/visualization/beetle_mass_reference.yaml')
        for variant,p in presets['presets'].items():
            m = load_model(str(ROOT/presets['model']),variant,ROOT,p['arguments'])
            self.assertTrue(m.links)
            self.assertTrue(set(p['initial_members']) <= set(m.links))
            if variant == 'roll_yaw':
                self.assertAlmostEqual(m.links['roll_joint_asm_link'].mass,.28288)
                self.assertAlmostEqual(m.joints['joint_roll'].pose[2,3],-.068)
                np.testing.assert_allclose(m.joints['pitchpipe_joint'].pose[:3,:3],transform(rpy=[0,0,3.142])[:3,:3])
                poses = m.fk('root')
                ref = registry.resolve(m,'roll_joint_asm_link')
                np.testing.assert_allclose(ref['reference']['cog_local_m'],[.04757,0,-.0311])
                # Explicitly transform back to roll link; ancestor transforms must cancel once.
                hand_in_roll = point(inverse(poses['roll_joint_asm_link']) @ poses['roll_hand_link'],m.links['roll_hand_link'].inertial[:3,3])
                np.testing.assert_allclose(hand_in_roll,[.18502,-.02352,-.05972])
            if variant in ('pitch_only','pitch_roll','pitch_yaw'):
                self.assertGreater(len(m.roots),1)


try:
    from cog_tools.scene import Scene, tf_name
    from visualization_msgs.msg import Marker
    from cog_tools.runtime import Runtime
    import rospy
    MESSAGES_AVAILABLE = True
except ImportError:
    MESSAGES_AVAILABLE = False


@unittest.skipUnless(MESSAGES_AVAILABLE,'ROS message classes unavailable (no node is initialized)')
class OfflineAdapterTests(unittest.TestCase):
    setUp = CoreTests.setUp
    def test_marker_lifecycle_axes_and_instances(self):
        class Publisher:
            def publish(self, msg):
                self.last = msg
        pub = Publisher()
        scene = Scene(pub,'fixture1')
        self.s.visible = {'roll_joint_asm_link','roll_hand_link'}
        report = self.s.evaluate(self.s.preview_poses(),1.25)
        scene.render(self.s,self.s.preview_poses(),report)
        ids = {(m.ns,m.id) for m in pub.last.markers}
        self.assertTrue(any(m.ns.endswith('mesh 0/0') for m in pub.last.markers))
        self.assertFalse(any('CAD (saved)' in m.ns for m in pub.last.markers))
        self.assertEqual(len(ids),len(pub.last.markers))
        self.assertTrue(all(m.header.stamp.to_sec()==1.25 for m in pub.last.markers))
        self.s.visible.remove('roll_hand_link')
        scene.render(self.s,self.s.preview_poses(),report)
        self.assertTrue(any(m.action==Marker.DELETE for m in pub.last.markers))
        self.assertFalse(any(m.action==Marker.DELETEALL for m in pub.last.markers))
        scene.legacy(self.model,'/gimbalrotor/',{'roll_joint_asm_link'})
        active = [m for m in pub.last.markers if m.action==Marker.ADD]
        self.assertTrue(all(m.frame_locked for m in active))
        self.assertTrue(all(m.header.frame_id=='gimbalrotor/roll_joint_asm_link' for m in active))
        self.assertEqual(tf_name('gimbalrotor','/gimbalrotor/fc'),'gimbalrotor/fc')
        self.assertNotIn('python_qt_binding',sys.modules)
        oriented = Model(simple_xml().replace('.02057 0 -.03110" rpy="0 0 0','.02057 0 -.03110" rpy="0 0 1.5707963267948966'),ROOT,'roll_yaw')
        scene.legacy(oriented,'',{'roll_joint_asm_link'})
        x_axis = next(m for m in pub.last.markers if m.ns.endswith('inertial (URDF)/0') and m.action==Marker.ADD)
        np.testing.assert_allclose([x_axis.points[1].x-x_axis.points[0].x,x_axis.points[1].y-x_axis.points[0].y,x_axis.points[1].z-x_axis.points[0].z],[0,.06,0],atol=1e-12)

    def test_coherent_live_missing_paused_and_backward_time(self):
        runtime = Runtime.__new__(Runtime)
        runtime.session, runtime.description_diagnostic = self.s, ''
        self.s.pose_source,self.s.fixed_frame = 'live','world'
        class Clock:
            current = 10
            @classmethod
            def now(cls):
                return rospy.Time.from_sec(cls.current)
            from_sec = staticmethod(rospy.Time.from_sec)
        runtime.rospy = types.SimpleNamespace(Time=Clock)
        runtime.last_ros_time,runtime.last_progress_wall = 10,time.monotonic()-1
        class Buffer:
            queries = []
            def lookup_transform(self, target, source, stamp):
                self.queries.append((target,source,stamp.to_sec()))
                if source=='roll_hand_link':
                    raise ValueError('stale dynamic TF')
                return types.SimpleNamespace(transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=0,y=0,z=0),rotation=types.SimpleNamespace(x=0,y=0,z=0,w=1)))
            def diagnostics(self):
                return []
        runtime.buffer = Buffer()
        self.s.preview_poses = lambda: self.fail('FK must not run in live TF mode')
        poses,stamp,diagnostics = runtime.poses()
        self.assertEqual({q[2] for q in runtime.buffer.queries},{9.9})
        self.assertTrue(any('paused' in d for d in diagnostics))
        self.assertIsNone(self.s.evaluate(poses,stamp,diagnostics)['aggregates']['declared']['cog_fixed_m'])
        Clock.current = 5
        resets = []
        runtime.create_listener = lambda: resets.append(True)
        poses,stamp,diagnostics = runtime.poses()
        self.assertEqual(poses,{})
        self.assertEqual(resets,[True])
        self.assertTrue(any('Backward' in d for d in diagnostics))


class IsolationTests(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec('roslaunch'),'roslaunch parser unavailable')
    def test_rviz_launch_substitutions_without_starting_nodes(self):
        from roslaunch.substitution_args import resolve_args
        root = ET.parse(ROOT/'launch/module_cog_viewer.launch').getroot()
        expr = root.find("arg[@name='rviz_fixed_frame']").get('value')
        node = root.find("node[@type='rviz']")
        prefix = node.get('launch-prefix')
        self.assertEqual(resolve_args(prefix,context={'arg':{'software_rendering':'true'}}),'env __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1')
        self.assertEqual(resolve_args(prefix,context={'arg':{'software_rendering':'false'}}),'')
        for mode in ('preview','live'):
            args = {'instance':'inspect_roll','pose_source':mode,'reference_frame':'gimbalrotor/fc'}
            args['rviz_fixed_frame'] = resolve_args(expr,context={'arg':args})
            resolved = resolve_args(node.get('args'),context={'arg':args})
            self.assertTrue(resolved.endswith('-f '+('cog_viz_inspect_roll/world' if mode=='preview' else 'gimbalrotor/fc')))

    def test_launch_xml_and_publisher_scope(self):
        for name in ('cog_markers','module_cog_viewer'):
            root = ET.parse(ROOT/('launch/'+name+'.launch')).getroot()
            self.assertFalse(root.findall('.//include'))
            self.assertFalse(any('robot_description'==p.get('name') for p in root.findall('.//param')))
        runtime = (ROOT/'scripts/visualization/cog_tools/runtime.py').read_text()
        self.assertNotIn('TransformBroadcaster',runtime)
        self.assertNotIn('set_param(',runtime)
        self.assertNotIn('JointState',runtime)
        self.assertNotIn('DELETEALL',(ROOT/'scripts/visualization/cog_tools/scene.py').read_text())


if __name__=='__main__':
    unittest.main(verbosity=2)
