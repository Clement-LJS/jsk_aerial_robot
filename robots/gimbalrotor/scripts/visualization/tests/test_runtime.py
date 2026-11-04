#!/usr/bin/env python3
"""Opt-in real ROS/Qt/RViz checks on an isolated ROS master, never robot bringup.

Run from an already sourced ROS terminal with desktop access:
  python3 -B scripts/visualization/tests/test_runtime.py
Writes logs/screenshots under exports/cog_viewer/runtime_validation. Starts and
stops only its own process groups. Mesa is selected for this test process only.
"""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import traceback
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts/visualization'))
ARTIFACTS = ROOT/'exports/cog_viewer/runtime_validation'


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=4)


def pump(app, seconds=.4):
    until = time.monotonic()+seconds
    while time.monotonic() < until:
        app.processEvents()
        time.sleep(.01)


def check(condition, message):
    if not condition:
        raise AssertionError(message)
    print('PASS:', message, flush=True)


def capture_pid(app, pid, destination):
    windows = subprocess.check_output(['wmctrl','-lp'],text=True)
    for row in windows.splitlines():
        values = row.split(None,4)
        if len(values) >= 3 and int(values[2]) == pid:
            subprocess.run(['wmctrl','-ia',values[0]],check=True)
            pump(app,.5)
            picture = app.primaryScreen().grabWindow(int(values[0],16))
            check(not picture.isNull() and picture.save(str(destination)), 'Captured '+destination.name)
            return
    raise AssertionError('No desktop window for PID '+str(pid))


def run():
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0))
        port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1',
                      ROS_HOME=str(ARTIFACTS/'ros_home'), ROS_LOG_DIR=str(ARTIFACTS/'ros_logs'),
                      __GLX_VENDOR_LIBRARY_NAME='mesa', LIBGL_ALWAYS_SOFTWARE='1')
    os.environ.pop('ROS_HOSTNAME',None)
    logs, processes = [], []
    results = {'status':'running','ros_master':os.environ['ROS_MASTER_URI'],'checks':[]}
    def spawn(command, name, env=None):
        log = open(ARTIFACTS/name,'w')
        logs.append(log)
        process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        processes.append(process)
        return process
    try:
        master_process = spawn(['roscore','-p',str(port)],'master.log')
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic()+20
        while True:
            try:
                master.getUri('/cog_runtime_test')
                break
            except OSError:
                if time.monotonic() > deadline or master_process.poll() is not None:
                    raise RuntimeError('Isolated ROS master failed; see master.log')
                time.sleep(.1)
        from python_qt_binding import QtCore, QtGui, QtWidgets
        app = QtWidgets.QApplication([])
        import rospy
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray
        rospy.init_node('cog_gui_integration',disable_signals=True)

        # Exercise the exact documented roslaunch, including both GUI processes.
        launch_env = os.environ.copy()
        launch_env.pop('__GLX_VENDOR_LIBRARY_NAME',None)
        launch_env.pop('LIBGL_ALWAYS_SOFTWARE',None)
        launch = spawn(['roslaunch','gimbalrotor','module_cog_viewer.launch','instance:=cog_runtime_smoke','preset:=roll_yaw'], 'launch.log',launch_env)
        status = rospy.wait_for_message('/cog_runtime_smoke/viewer/status',String,timeout=25)
        report = json.loads(status.data)
        check(report['aggregates']['declared']['status']=='complete', 'Documented launch publishes a complete declared aggregate')
        markers = rospy.wait_for_message('/cog_runtime_smoke/viewer/markers',MarkerArray,timeout=10)
        check(any(m.type==m.MESH_RESOURCE for m in markers.markers),'Documented launch publishes STL mesh markers')
        pump(app,4)
        for node,image in [('/cog_runtime_smoke/viewer','launched_inspector.png'),('/cog_runtime_smoke/rviz','launched_rviz.png')]:
            uri = master.lookupNode('/cog_runtime_test',node)[2]
            pid = xmlrpc.client.ServerProxy(uri).getPid('/cog_runtime_test')[2]
            if node.endswith('/rviz'):
                child_environment = Path('/proc/%d/environ' % pid).read_bytes().split(b'\0')
                check(b'__GLX_VENDOR_LIBRARY_NAME=mesa' in child_environment and b'LIBGL_ALWAYS_SOFTWARE=1' in child_environment,'Launch applies the Mesa fallback to RViz without shell environment overrides')
            capture_pid(app,pid,ARTIFACTS/image)
        check(launch.poll() is None,'Both launched windows stay alive')
        stop(launch)
        text = (ARTIFACTS/'launch.log').read_text()
        check('Unable to create' not in text and 'Traceback' not in text,'Launch log contains no GLX creation error or Python traceback')
        results['checks'].append('documented_launch_and_real_rviz_windows')

        from cog_tools.runtime import Runtime
        from cog_tools.gui import Window
        import numpy as np
        runtime = Runtime(gui=True)
        window = Window(runtime)
        window.show()
        pump(app)
        s = runtime.session
        window.tree.setCurrentItem(window.items['roll_joint_asm_link'])
        window.start_assembly()
        window.isolate()
        pump(app)
        check(s.visible==s.members=={'roll_joint_asm_link'},'Select / start assembly / isolate show one module')
        check(abs(s.last_report['aggregates']['declared']['mass_kg']-.28288)<1e-10,'Single-module URDF mass is 0.28288 kg')

        # Supply a choice through the actual Qt dialog rather than replacing the handler.
        def accept_child():
            dialog = app.activeModalWidget()
            if isinstance(dialog,QtWidgets.QInputDialog):
                dialog.setTextValue('roll_hand_link via joint_roll')
                dialog.accept()
        QtCore.QTimer.singleShot(100,accept_child)
        window.add_adjacent('child')
        pump(app)
        check(s.members=={'roll_joint_asm_link','roll_hand_link'},'Actual Add child dialog attaches the hand through joint_roll')
        check(abs(s.last_report['aggregates']['declared']['mass_kg']-.59094)<1e-10,'Two-body selected mass is 0.59094 kg')
        original = np.array(s.last_report['aggregates']['declared']['cog_fixed_m'])
        slider,number = window.joint_widgets['joint_roll']
        number.setValue(math.pi/2)
        pump(app)
        check(not np.allclose(original,s.last_report['aggregates']['declared']['cog_fixed_m']),'Joint numeric control moves the assembly CoG')
        slider.setValue(500)
        pump(app)
        check(abs(s.joints['joint_roll'])<1e-10,'Joint slider updates model state')
        agg = s.last_report['aggregates']['declared'].copy()
        window.items['roll_hand_link'].setCheckState(1,QtCore.Qt.Unchecked)
        pump(app)
        check(s.last_report['aggregates']['declared']==agg,'Hiding geometry leaves mass/CoG unchanged')
        window.items['roll_hand_link'].setCheckState(2,QtCore.Qt.Unchecked)
        pump(app)
        check(abs(s.last_report['aggregates']['declared']['mass_kg']-.28288)<1e-10,'Excluding hand mass changes membership')

        window.tree.setCurrentItem(window.items['roll_joint_asm_link'])
        before = s.last_report['rows']['roll_joint_asm_link']
        window.visual_xyz.setText('.037 0 0')
        window.apply_visual()
        pump(app)
        after = s.last_report['rows']['roll_joint_asm_link']
        for mode in ('declared','reference'):
            check(np.allclose(before[mode]['cog_local_m'],after[mode]['cog_local_m']),mode+' CoG stays fixed after visual-only edit')
        check(np.allclose(np.array(after['candidate']['cog_local_m'])-before['candidate']['cog_local_m'],[.01,0,0]),'Mesh-follow candidate moves by 10 mm')
        window.origin_joint.setCurrentText('joint_yaw')
        window.origin_xyz.setText('.2505 0 0')
        window.apply_origin()
        pump(app)
        check(not np.allclose(after['declared']['cog_fixed_m'],s.last_report['rows']['roll_joint_asm_link']['declared']['cog_fixed_m']),'Joint-origin editor moves the attached subtree')
        window.export()
        session_file = s.export_folder()/'session.yaml'
        import yaml
        s.restore(yaml.safe_load(session_file.read_text()))
        window.rebuild()
        pump(app)
        check(('roll_joint_asm_link',0) in s.visual_poses and 'joint_yaw' in s.joint_origins,'Actual export / restore preserves preview overrides')
        window.reset_overrides()
        results['checks'].append('gui_select_attach_joint_visibility_mass_origins_export_restore')

        for variant in runtime.presets['presets']:
            window.preset.setCurrentText(variant)
            window.reload_model()
            pump(app,.15)
            check(s.inspected in runtime.last_poses,'Preset '+variant+' opens its selected module with an available pose')
            if variant in ('pitch_only','pitch_roll','pitch_yaw'):
                check(len(s.model.roots)>1,'Preset '+variant+' reports its actual disconnected components')
        window.preset.setCurrentText('roll_yaw')
        window.reload_model()
        pump(app)
        generation = s.generation
        broken = ARTIFACTS/'invalid_reload.urdf'
        broken.write_text('<robot>broken')
        window.source.setText(str(broken))
        try:
            window.reload_model()
            raise AssertionError('Malformed URDF unexpectedly loaded')
        except ValueError:
            check(s.generation==generation,'Bad GUI reload retains the last valid generation')
        window.source.setText(runtime.source)
        results['checks'].append('all_preset_gui_loading_and_bad_reload_retention')

        # Real RViz Python widget permits inspecting the display status and pixels.
        import rviz
        rv = rviz.bindings.VisualizationFrame()
        rv.setSplashPath('')
        rv.initialize()
        config = rviz.bindings.Config()
        rviz.bindings.YamlConfigReader().readFile(config,str(ROOT/'config/visualization/module_cog_viewer.rviz'))
        rv.load(config)
        manager = rv.getManager()
        manager.startUpdate()
        manager.setFixedFrame(s.fixed_frame)
        display = manager.getRootDisplayGroup().getDisplayAt(0)
        display.subProp('Marker Topic').setValue(rospy.resolve_name('~markers'))
        window.s.overlays.update(reference=False,candidate=False,ghost=False,joints=False)
        window.rebuild()
        rv.resize(1100,800)
        rv.show()
        pump(app,3)
        rv.raise_()
        rv.activateWindow()
        pump(app,1)
        check(app.primaryScreen().grabWindow(int(rv.winId())).save(str(ARTIFACTS/'rviz_verified.png')),'Saved actual rendered RViz scene')
        check(window.grab().save(str(ARTIFACTS/'inspector_verified.png')),'Saved actual interactive inspector')
        statuses, root_status = [], ''
        for idx in range(display.numChildren()):
            prop = display.childAt(idx)
            print('RViz property:',prop.getName(),str(prop.getValue()),flush=True)
            if not prop.getName().startswith('Status'):
                continue
            root_status = prop.getName()
            status_property = prop
            for idx in range(status_property.numChildren()):
                p = status_property.childAt(idx)
                statuses.append({'name':p.getName(),'value':str(p.getValue())})
        results['rviz_marker_status'] = statuses
        results['rviz_display_status'] = root_status
        check(root_status=='Status: Ok' and bool(statuses),'RViz MarkerArray display reports OK, including mesh loading')
        results['checks'].append('real_rviz_render_and_marker_status')

        # Route actual Qt mouse events through RViz's render panel and active tool.
        tools = manager.getToolManager()
        check(tools.getCurrentTool().getClassId()=='rviz/MoveCamera','Move Camera is the active startup tool')
        check(any(tools.getTool(i).getClassId()=='rviz/FocusCamera' for i in range(tools.numTools())),'Focus Camera is available in the toolbar')
        # RenderPanel's C++ accessor is not exposed by all ROS1 Python bindings.
        panel = next(child for child in rv.findChildren(QtWidgets.QWidget) if child.metaObject().className()=='rviz::RenderPanel')
        view = manager.getViewManager().getCurrent()
        panel.setFocus()
        center = QtCore.QPointF(panel.width()/2,panel.height()/2)
        aggregate_before_camera = s.last_report['aggregates']['declared'].copy()
        def drag(button, modifiers=QtCore.Qt.NoModifier):
            end = center+QtCore.QPointF(70,35)
            for kind,pos,changed,held in [(QtCore.QEvent.MouseButtonPress,center,button,button),
                                           (QtCore.QEvent.MouseMove,end,QtCore.Qt.NoButton,button),
                                           (QtCore.QEvent.MouseButtonRelease,end,button,QtCore.Qt.NoButton)]:
                event = QtGui.QMouseEvent(kind,pos,changed,held,modifiers)
                app.sendEvent(panel,event)
                pump(app,.08)
        yaw = float(view.subProp('Yaw').getValue())
        drag(QtCore.Qt.LeftButton)
        check(abs(float(view.subProp('Yaw').getValue())-yaw)>1e-4,'Left mouse drag rotates the real RViz camera')
        def focal_point():
            prop = view.subProp('Focal Point')
            return np.array([float(prop.subProp(axis).getValue()) for axis in ('X','Y','Z')])
        focal = focal_point()
        drag(QtCore.Qt.MiddleButton)
        check(not np.allclose(focal,focal_point()),'Middle mouse drag pans the real RViz camera')
        focal = focal_point()
        drag(QtCore.Qt.LeftButton,QtCore.Qt.ShiftModifier)
        check(not np.allclose(focal,focal_point()),'Shift + left mouse drag pans the real RViz camera')
        distance = float(view.subProp('Distance').getValue())
        drag(QtCore.Qt.RightButton)
        check(abs(float(view.subProp('Distance').getValue())-distance)>1e-4,'Right mouse drag zooms the real RViz camera')
        distance = float(view.subProp('Distance').getValue())
        wheel = QtGui.QWheelEvent(center,QtCore.QPointF(panel.mapToGlobal(center.toPoint())),QtCore.QPoint(),QtCore.QPoint(0,120),QtCore.Qt.NoButton,QtCore.Qt.NoModifier,QtCore.Qt.NoScrollPhase,False)
        app.sendEvent(panel,wheel)
        pump(app,.2)
        check(abs(float(view.subProp('Distance').getValue())-distance)>1e-4,'Mouse wheel zooms the real RViz camera')
        tools.setCurrentTool(next(tools.getTool(i) for i in range(tools.numTools()) if tools.getTool(i).getClassId()=='rviz/Select'))
        pump(app,.2)
        move_button = next(button for button in rv.findChildren(QtWidgets.QToolButton) if button.isVisible() and button.text().replace('&','').startswith('Move Camera'))
        move_button.click()
        pump(app,.2)
        check(tools.getCurrentTool().getClassId()=='rviz/MoveCamera','Move Camera toolbar action returns from selection to camera navigation')
        check(s.last_report['aggregates']['declared']==aggregate_before_camera,'Mouse camera navigation leaves mass/CoG unchanged')
        results['checks'].append('real_rviz_mouse_rotate_pan_zoom_and_toolbar')

        # Read-only live adapter against test-only static TF on the isolated master.
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from cog_tools.math3d import quaternion
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        transforms = []
        expected = s.preview_poses()
        expected_cog = np.array(s.last_report['aggregates']['declared']['cog_fixed_m'])
        for name,t in expected.items():
            msg = TransformStamped()
            msg.header.frame_id, msg.child_frame_id = 'validation_world','validation/'+name
            msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z = t[:3,3]
            msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w = quaternion(t)
            transforms.append(msg)
        s.tf_prefix = 'validation'
        runtime.set_mode('live','validation_world')
        broadcaster.sendTransform(transforms)
        pump(app,1)
        runtime.tick()
        check(s.last_report['aggregates']['declared']['status']=='complete','Real TF listener accepts zero-stamped static TF at the common query time')
        check(np.allclose(s.last_report['aggregates']['declared']['cog_fixed_m'],expected_cog),'Live static TF places the CoG exactly once')
        window.rebuild()
        check(not window.joint_panel.isEnabled() and not window.edit_panel.isEnabled(),'Live mode disables joint/visual/origin editing')
        s.tf_prefix = 'nonexistent_validation'
        runtime.tick()
        check(s.last_report['aggregates']['declared']['status']=='incomplete','Missing real TF suppresses the complete aggregate')
        import threading
        dynamic = tf2_ros.TransformBroadcaster()
        done = threading.Event()
        def publish_dynamic():
            import copy
            while not done.is_set():
                messages = copy.deepcopy(transforms)
                stamp = rospy.Time.now()
                for msg in messages:
                    msg.child_frame_id = msg.child_frame_id.replace('validation/','dynamic_validation/')
                    msg.header.stamp = stamp
                    msg.transform.translation.x += .1
                    msg.transform.translation.y += .02
                    msg.transform.translation.z -= .03
                dynamic.sendTransform(messages)
                done.wait(.02)
        thread = threading.Thread(target=publish_dynamic,daemon=True)
        s.tf_prefix = 'dynamic_validation'
        thread.start()
        try:
            pump(app,1)
            check(s.last_report['aggregates']['declared']['status']=='complete','Real dynamic TF produces a coherent complete live aggregate')
            check(np.allclose(s.last_report['aggregates']['declared']['cog_fixed_m'],expected_cog+[.1,.02,-.03]),'Live dynamic TF moves the CoG by the published translation')
        finally:
            done.set()
            thread.join(timeout=2)
        pump(app,.5)
        check(s.last_report['aggregates']['declared']['status']=='incomplete','Stopped dynamic TF becomes stale and suppresses the aggregate')
        for msg in transforms:
            msg.child_frame_id = msg.child_frame_id.replace('validation/','dynamic_validation/')
            msg.header.stamp = rospy.Time.from_sec(rospy.Time.now().to_sec()+10)
        dynamic.sendTransform(transforms)
        pump(app,.2)
        check(any('future' in d for d in runtime.buffer.diagnostics()),'Future dynamic TF is rejected and reported')
        results['checks'].append('real_tf_static_dynamic_missing_stale_future_live_observation')

        # Exercise both other public entry points through the built package.
        headless = spawn(['roslaunch','gimbalrotor','cog_markers.launch','instance:=cog_headless_smoke'],'headless.log')
        headless_status = json.loads(rospy.wait_for_message('/cog_headless_smoke/markers/status',String,timeout=20).data)
        check(abs(headless_status['aggregates']['declared']['mass_kg']-.59094)<1e-10,'Built headless entry point publishes the two-body aggregate')
        stop(headless)
        rospy.set_param('/validation_description',s.model.xml)
        legacy = spawn(['rosrun','gimbalrotor','urdf_inertia_markers.py','__name:=cog_legacy_smoke','_robot_description_param:=/validation_description','_tf_prefix:=validation','_only_links:=roll_joint_asm_link'],'legacy.log')
        legacy_markers = rospy.wait_for_message('/urdf/inertia_markers',MarkerArray,timeout=15)
        check(bool(legacy_markers.markers) and all(m.frame_locked and m.header.frame_id=='validation/roll_joint_asm_link' for m in legacy_markers.markers),'Built legacy entry point preserves topic/filter/prefix and frame locking')
        stop(legacy)
        results['checks'].append('built_headless_and_legacy_entry_points')
        rv.close()
        window.close()
        results['status'] = 'passed'
    except Exception:
        results['status'] = 'failed'
        results['error'] = traceback.format_exc()
        raise
    finally:
        if 'window' in locals():
            window.close()
        if 'rv' in locals():
            rv.getManager().stopUpdate()
            rv.setWindowModified(False)
            rv.close()
            # Dispose Qt/C++ subscriptions while the isolated master still exists.
            import sip
            sip.delete(rv)
        if 'rospy' in locals() and not rospy.is_shutdown():
            rospy.signal_shutdown('Runtime checks complete')
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        (ARTIFACTS/'result.json').write_text(json.dumps(results,indent=2))
        print('Artifacts:',ARTIFACTS,flush=True)


if __name__=='__main__':
    run()
