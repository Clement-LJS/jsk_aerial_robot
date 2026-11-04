"""Read-only ROS pose adapter and entry point; Qt loaded only for interactive mode."""
from pathlib import Path
import json
import time
import re
import threading
import numpy as np
import yaml
from .model import load_model
from .reference import Registry
from .session import Session
from .math3d import from_quaternion


def project_root():
    # Source-tree execution and installed helper directory both work without cwd assumptions.
    for parent in Path(__file__).resolve().parents:
        if (parent / 'config/visualization/beetle_viewer_presets.yaml').is_file():
            return parent
    import rospkg
    return Path(rospkg.RosPack().get_path('gimbalrotor'))


class Runtime:
    def __init__(self, gui=False, legacy=False):
        import rospy
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray
        from .scene import Scene
        self.rospy, self.String = rospy, String
        self.root = project_root()
        self.presets = yaml.safe_load((self.root/'config/visualization/beetle_viewer_presets.yaml').read_text())
        self.preset_name = rospy.get_param('~preset', 'roll_yaw')
        self.source_override = rospy.get_param('~model_source', '')
        self.robot_description = rospy.get_param('~robot_description_param', '/robot_description' if legacy else '/gimbalrotor/robot_description')
        self.mode = rospy.get_param('~pose_source', 'preview')
        self.legacy = legacy
        self.source = self.source_override or ('param:'+self.robot_description if self.mode == 'live' or legacy else str(self.root/self.presets['model']))
        model = self.read_model()
        self.reference_path = Path(rospy.get_param('~reference_file', str(self.root/'config/visualization/beetle_mass_reference.yaml')))
        scene_id = rospy.get_param('~scene_id', None)
        if scene_id is not None and not re.fullmatch(r'[A-Za-z0-9_]+', scene_id):
            raise ValueError('scene_id must contain only letters, digits and underscores')
        self.session = Session(model, Registry.read(self.reference_path), self.root, scene_id)
        s = self.session
        s.tf_prefix = rospy.get_param('~tf_prefix', '')
        s.latency, s.max_pose_age = float(rospy.get_param('~latency', .1)), float(rospy.get_param('~max_pose_age', .5))
        if not 0 <= s.latency <= s.max_pose_age or not np.isfinite([s.latency, s.max_pose_age]).all():
            raise ValueError('Require 0 <= latency <= max_pose_age, finite')
        s.overlays['candidate'] = bool(rospy.get_param('~show_candidate', False))
        s.threshold_m = float(rospy.get_param('~discrepancy_threshold_mm',1))/1000
        if not np.isfinite(s.threshold_m) or s.threshold_m < 0:
            raise ValueError('Discrepancy threshold must be finite and nonnegative')
        only = rospy.get_param('~only_links', '')
        self.only_links = set(only if isinstance(only, list) else [n.strip() for n in only.split(',') if n.strip()])
        missing = self.only_links-set(model.links)
        if missing:
            rospy.logwarn('Requested links unavailable: %s', sorted(missing))
        preset = self.presets['presets'].get(self.preset_name, {})
        s.members = self.only_links & set(model.links) if self.only_links else set(preset.get('initial_members', [])) & set(model.links)
        s.visible = set(s.members) or set(model.links)
        if s.members:
            s.inspected = sorted(s.members)[0]
            s.root = model.root_of(s.inspected)
        s.fixed_frame = rospy.get_param('~reference_frame', '') or s.fixed_frame
        self.listener, self.buffer, self.last_ros_time, self.last_progress_wall = None, None, None, time.monotonic()
        self.reload_token = rospy.get_param('~reload_token', 0)
        self.last_poll_wall, self.description_diagnostic = 0, ''
        self.last_poses, self.last_status_wall = {}, 0
        self.set_mode(self.mode, rospy.get_param('~reference_frame', '') or 'gimbalrotor/base_link')
        self.marker_topic = rospy.get_param('~marker_topic', '/urdf/inertia_markers' if legacy else '~markers')
        self.scene = Scene(rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True), s.instance)
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.rate = float(rospy.get_param('~rate', 25))
        if not np.isfinite(self.rate) or not 1 <= self.rate <= 60:
            raise ValueError('Viewer rate must be between 1 and 60 Hz')
        rospy.on_shutdown(self.scene.clear)

    def read_model(self):
        preset = self.presets['presets'].get(self.preset_name)
        if preset is None:
            raise ValueError('Unknown variant preset: '+self.preset_name)
        return load_model(self.source, self.preset_name, self.root, preset['arguments'])

    def reload(self):
        try:
            new_model = self.read_model()
            # Validate registry independently; do not reimport/reseed it from the changed URDF.
            Registry(self.session.registry.data)
            self.session.replace_model(new_model)
        except Exception as exc:
            self.session.last_error = 'Reload rejected; retaining last valid generation: '+str(exc)
            raise

    def set_mode(self, mode, frame=None):
        if mode not in ('preview', 'live'):
            raise ValueError('pose_source must be preview or live')
        s = self.session
        if mode == 'live':
            if s.joint_origins or s.visual_poses or s.joints:
                raise ValueError('Reset preview overrides before live observation')
            if self.listener is None:
                self.create_listener()
            s.fixed_frame = (frame or 'gimbalrotor/base_link').strip('/')
            if not s.fixed_frame:
                raise ValueError('A nonempty live reference frame is required')
        else:
            s.fixed_frame = 'cog_viz_'+s.instance+'/world'
            if s.pose_source != 'preview':
                s.pose_note = 'Offline model FK; joint values reset to model defaults (use Copy live snapshot to retain a live pose)'
        self.mode = s.pose_source = mode

    def create_listener(self):
        import tf2_ros
        rospy = self.rospy
        class GuardedBuffer(tf2_ros.Buffer):
            def __init__(self):
                super().__init__(debug=False)
                self.health_lock, self.rejected = threading.Lock(), {}

            def set_transform(self, message, authority):
                # TF callbacks only touch the protected buffer/health state, never session/Qt.
                now = rospy.Time.now().to_sec()
                stamp = message.header.stamp.to_sec()
                child = message.child_frame_id
                with self.health_lock:
                    if stamp <= 0 or stamp > now+.05:
                        self.rejected[child] = 'Rejected zero/future dynamic TF %s (stamp %.6f, now %.6f)' % (child, stamp, now)
                        return
                    self.rejected.pop(child, None)
                super().set_transform(message, authority)

            def diagnostics(self):
                with self.health_lock:
                    return list(self.rejected.values())
        if self.listener is not None:
            self.listener.unregister()
        self.buffer = GuardedBuffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

    def poses(self):
        from .scene import tf_name
        s, rospy = self.session, self.rospy
        now, wall, diagnostics = rospy.Time.now().to_sec(), time.monotonic(), []
        if s.pose_source == 'preview':
            return s.preview_poses(), now, diagnostics
        if self.last_ros_time is not None and now < self.last_ros_time:
            # Recreate listener, including /tf_static subscription, after a time reset.
            self.create_listener()
            self.last_ros_time, self.last_progress_wall = now, wall
            return {}, max(0, now), ['Backward ROS clock jump: TF cache reset; waiting for consistent poses']
        if self.last_ros_time != now:
            self.last_progress_wall = wall
        elif wall-self.last_progress_wall > s.max_pose_age:
            diagnostics.append('ROS clock paused; displaying paused-time poses')
        self.last_ros_time = now
        stamp = now-s.latency
        if stamp <= 0:
            return {}, max(0, now), ['ROS time unavailable/too early for requested latency']
        poses = {}
        for name in s.model.links:
            try:
                msg = self.buffer.lookup_transform(s.fixed_frame, tf_name(s.tf_prefix, name), rospy.Time.from_sec(stamp))
                p, q = msg.transform.translation, msg.transform.rotation
                poses[name] = from_quaternion([p.x,p.y,p.z], [q.x,q.y,q.z,q.w])
            except Exception as exc:
                diagnostics.append('Missing/stale TF %s at %.6f: %s' % (name, stamp, exc))
        # Exact-time queries enforce one snapshot; TF extrapolation rejects stale dynamic data.
        # Static TF needs no age rejection, and is valid at every requested time.
        diagnostics.extend(self.buffer.diagnostics())
        if self.description_diagnostic:
            diagnostics.append(self.description_diagnostic)
        diagnostics.append('Live TF is pose authority; model/TF calibration agreement is unverified')
        return poses, stamp, diagnostics

    def tick(self):
        wall = time.monotonic()
        if wall-self.last_poll_wall > .5:
            self.last_poll_wall = wall
            token = self.rospy.get_param('~reload_token', 0)
            if token != self.reload_token:
                self.reload_token = token
                try:
                    self.reload()
                except Exception as exc:
                    self.rospy.logwarn('%s', exc)
            self.description_diagnostic = ''
            if self.session.model.source.startswith('param:') and self.session.pose_source == 'live':
                try:
                    from .model import digest
                    if digest(self.rospy.get_param(self.session.model.source[6:]).encode()) != self.session.model.fingerprint:
                        self.description_diagnostic = 'External robot_description changed; reload required. TF/model agreement is unverified.'
                except Exception as exc:
                    self.description_diagnostic = 'Description freshness unavailable: '+str(exc)
        if self.legacy:
            self.scene.legacy(self.session.model, self.session.tf_prefix, self.only_links)
            return None
        try:
            poses, stamp, diagnostics = self.poses()
        except Exception as exc:
            poses, stamp, diagnostics = {}, max(0, self.rospy.Time.now().to_sec()), ['Pose unavailable: '+str(exc)]
        report = self.session.evaluate(poses, stamp, diagnostics)
        self.scene.render(self.session, poses, report)
        self.last_poses = {k:v.copy() for k,v in poses.items()}
        if time.monotonic()-self.last_status_wall >= .2:
            self.status_pub.publish(self.String(data=json.dumps(report, allow_nan=False)))
            self.last_status_wall = time.monotonic()
        return report


def main(gui=False, legacy=False):
    import rospy
    rospy.init_node('module_cog_viewer' if gui else ('urdf_inertia_markers' if legacy else 'cog_markers'), anonymous=False)
    try:
        runtime = Runtime(gui, legacy)
        if gui:
            from .gui import run
            run(runtime)
        else:
            # Wall-time pacing keeps status/reload handling responsive when simulated time pauses.
            while not rospy.is_shutdown():
                try:
                    runtime.tick()
                except Exception as exc:
                    runtime.scene.clear()
                    rospy.logerr_throttle(2, 'Viewer snapshot unavailable: %s', exc)
                time.sleep(1/runtime.rate)
    except Exception as exc:
        rospy.logfatal('CoG viewer: %s', exc)
        raise
