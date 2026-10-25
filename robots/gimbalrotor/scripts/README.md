**Last updated:** 2026-08-17  
**Documented branch:** `develop/pochitab_passiveperching/main`  

## Keyboard command

### `keyboard_roll_yaw_perching.py`

Start Gazebo in one terminal:

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch gimbalrotor roll_yaw_perching.launch
```

For hardware, use `roslaunch gimbalrotor roll_yaw_perching.launch simulation:=false`
instead. Hardware uses the configured Spinal serial bridge and OptiTrack; physical
contact must already exist before enabling perching. Gazebo creates an ideal
passive hinge when perching is enabled.

Start the keyboard in a second terminal:

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
rosrun gimbalrotor keyboard_roll_yaw_perching.py
```

Press a motion key, type a number in **degrees**, then Enter. Wait for **Done**
before the next move. Example: `r`, `t`, wait for HOVER, `e`, `y` → `30` → Enter,
wait, `o` → `30` → Enter, wait, `u` → `10` → Enter, wait, `c` → `20` → Enter.

| Key | Action |
| --- | --- |
| `r`, `t`, `e` | Arm, take off, enable perching |
| `y` | Enter desired body yaw relative to the contact lock |
| `o` | Enter desired body roll relative to the contact lock |
| `u` | Enter a positive amount to raise local pitch |
| `c` | Enter a positive amount for the downward local pitch stroke |
| `v` | Set pitch speed in degrees/second (default 1.72) |
| Space | Pause/resume the current motion |
| `x` | Cancel the motion and hold contact |
| `s`, `?` | Status, help |
| `d` | Disable perching while holding; releases the Gazebo fixture |
| `l` | Land after perching has been disabled and physical contact released |
| `h` | Halt motors |
| `q`, Ctrl+C | Quit; request cancellation of this keyboard's unfinished move |

Yaw/roll are body targets, not direct joint positions. Pitch-up decreases local
pitch; downward cutting increases it. Thus `u 10` followed by `c 20` moves from
0° to −10°, then to +10°. Pitch is about the aligned body-local Y axis, with the
same fixed contact maintained through coordinated active joints, passive hinge,
body orientation and CoG position. It is not a world-vertical translation.

The script handles session identities, preparation, alignment and execution.
It can reconnect to an existing settled lock using the navigator's current
target angles. It never publishes direct servo or free-flight pose commands.
The navigator retains all feasibility/tracking checks. No saw motor is actuated.

Verified through the actual keyboard process in headless Gazebo: arm/takeoff,
contact lock, yaw 30°, roll 30°, pitch up 10°, then downward pitch 20° with
pause/resume. A restarted keyboard adopted the held pose, moved to roll 45°,
and passed pitch-up, cancellation and quit-during-planning checks. The upward
stroke changed both active joint angles, estimated passive angle, body attitude
and body position. Peak sampled pivot/body tracking errors were 5.5 mm / 0.88°.
The rebuilt planner passed 15 checks and the keyboard state handling passed seven.
Hardware launch parameters were checked without starting hardware nodes.

### `keyboard_changerpy.py`

Change robot's rpy using keyboard. 

### `keyboard_command2.py`

Same as: keyboard_command.py 

- except: track flight state (prevent arming when it is already in hovering state...)

### `keyboard_perching_pitch.py`

Manually changes the pitch delta relative to the locked perching orientation.

### `keyboard_servo.py`

Moves one servo by relative encoder steps while using the latest measured servo angle as the command base.

Run with:  
rosrun gimbalrotor keyboard_servo.py _servo_index:=4

### `saw_control.py`

Publish saw pwm with keyboard

## Perching geometry script

### `perching.py`

Manual publish robot's perching pith (using robot's pitch joint, robot's orientation, robot's position)

## Tasks

### `auto_perching_pitch_experiment.py`

Runs a repeatable downward perching-pitch step experiment.

### `close_hand_then_lock.py`

Safely performs the ordered mechanical sequence: close the hand first, then close the lock.

-- Todo, delete if unnecessary

### `servo_pidgain.py`

Applies and verifies one DYNAMIXEL servo's position PID gains.

To check for servo pid, run:  
rosservice call /get_board_info

## URDF 

### `urdf_inertia_markers.py`

Visualizes URDF inertial properties in RViz.

## Mujoco_scripts 

### `postprocess_beetle_cutting_model.py`
Post-processes the MuJoCo model by positioning the robot at the configured perch pivot and adding the cutting contact site, perching constraints, and branch visualization.

## Multilink passive-pivot pitch rezero

`keyboard_command/keyboard_perching_secondary.py` and
`tasks/auto_multilink_perching_pitch_experiment.py` use the multilink navigator
with the normal `gimbalrotor_controller`. The active mechanism stays exactly
`joint_pitch` plus one yaw/roll joint. The passive hinge is external to URDF/KDL.

Calibrate `pitch_rezero_pivot_offset_{x,y,z}` in
`config/beetle/MultilinkPerching.yaml` from CAD/measurement: the default assumes
that `yaw_hand_link` origin is the physical hinge center. The axis is also in
that contact-link frame (default +Y). At the trigger, the measured BASELINK pose
and measured mechanism FK give `T_W_C = T_W_B * T_B_C`. The navigator stores
`P = T_W_C * offset` and `a = normalize(R_W_C * axis)`. During rezero it commands
`R_d = Rot(a, alpha) * R_0` and `p_d = P + Rot(a, alpha) * (p_0 - P)`, keeping
both active joints and the body-to-CoG vector frozen. The ordinary fixed-contact
cutting relation `T_W_B = locked_contact_world * inverse(T_B_C(q))` resumes only
after reset/relock. Relocking is a software assumption of fixed contact; it does
not mechanically secure the passive hinge.

The rate-limited outer loop uses measured BASELINK world pitch. Configure
`pitch_rezero_command_sign` for the actual direction of pitch response. An
arbitrary hinge orientation may make zero pitch unreachable within the allowed
30-degree travel; this results in a timeout hold. Successful stabilization needs
pitch error within 2 degrees and pitch rate within 0.05 rad/s continuously for
0.30 seconds. Both success and failure hold the last valid body target with the
active joints frozen. Failure never returns to the old tilted lock automatically.
Disable/retry safely after failure. A feedback gap over 0.1 s breaks stabilization;
a clock reversal or gap over 0.5 s fails the maneuver. Invalid configuration
(non-finite/range-invalid values, zero axis/sign, timeout shorter than dwell)
disables the rezero subscriber. Invalid/stale joint diagnostics and unavailable
pitch-rate estimates are published as NaN.

During ACTIVE and HOLD (ready or failed), measured active pitch and secondary
joint positions are checked against their values captured at rezero start using
wrapped angular errors. `pitch_rezero_pitch_joint_hold_tolerance` and
`pitch_rezero_secondary_joint_hold_tolerance` each default to `0.02` rad (about
1.15 degrees) and must be finite and in `(0, 0.5]` rad. Exceeding either tolerance
invalidates the passive-pivot-only motion assumption and enters failure HOLD;
stale/unavailable mechanism feedback also fails and publishes both hold errors
as NaN. Failure clears READY, stops further passive rotation commands, and keeps
commanding the original frozen joints and last valid body target. Monitoring
continues until reset/relock. The scripts check READY/FAILED before relocking;
the raw `/perching/relock` callback retains its existing reset behavior and does
not enforce these flags.

Manual keyboard:

```bash
rosrun gimbalrotor keyboard_perching_secondary.py
```

Physically perch first. Press `e` for a fresh provisional lock, `z` for whole-body
rezero, `s` to check ready, and `r` for a fresh final lock. Then `j/l` decreases/
increases the secondary joint, `i/k` increases/decreases the logical cutting
pitch delta, and Space returns both joints to the final locked configuration.
`d` disables perching, `h` shows help, and Ctrl-C exits without changing robot
state. Motion is blocked during pending rezero, ACTIVE, and either HOLD outcome.
Secondary commands use physical joint coordinates divided by the configured
secondary sign; pitch diagnostics account for the configured pitch sign.
Both scripts wait for lock timestamps strictly newer than their request and for
subsequent nominal feedback. They publish enable and relock only once per request.

Equivalent topic sequence (default robot namespace):

```bash
# Physically perch, then provisionally lock.
rostopic pub -1 /gimbalrotor/perching/enable std_msgs/Bool 'data: true'
rostopic pub -1 /gimbalrotor/perching/multilink/pitch_rezero std_msgs/Empty '{}'
# Wait for true; check failed is false before relocking.
rostopic echo /gimbalrotor/perching/multilink/pitch_rezero_ready
rostopic pub -1 /gimbalrotor/perching/relock std_msgs/Empty '{}'
# After the fresh final lock, use the existing secondary/pitch command topics.
# Example zero logical pitch delta (returns pitch to final lock):
rostopic pub -1 /gimbalrotor/perching/manual_pitch_delta std_msgs/Float64 'data: 0.0'
# Secondary target is an absolute joint command in radians, divided by its sign:
# rostopic pub -1 /gimbalrotor/perching/multilink/secondary_joint_target std_msgs/Float64 'data: <target>'
rostopic pub -1 /gimbalrotor/perching/enable std_msgs/Bool 'data: false'
```

Automatic experiment (run only after physically perching in HOVER):

```bash
rosrun gimbalrotor auto_multilink_perching_pitch_experiment.py \
  _perform_pitch_rezero:=true _step_deg:=0.8 _hold_time:=15.0 _number_of_steps:=15
```

Default sequence: enable → fresh provisional lock → rezero → ready → relock →
fresh final lock → secondary settled → active cutting pitch steps. Rezero
failure or missing fresh feedback aborts without cutting. Failure after rezero
starts leaves the navigator holding; it does not silently disable or relock.
`~perform_pitch_rezero:=false` skips rezero/relock and uses the initial lock.
Other defaults are `~direction:=1`, `~max_delta_deg:=20`,
`~return_to_lock_at_end:=false`, `~disable_perching_at_end:=false`,
`~lock_timeout:=3.0`, `~rezero_timeout:=8.0`,
`~secondary_settle_timeout:=3.0`, `~target_update_timeout:=1.0`, and
`~feedback_timeout:=0.5`. The keyboard lock timeout defaults to 2.0 s. Both
scripts support `~robot_namespace` (default `/gimbalrotor`). Wait deadlines use
wall time so paused ROS time cannot hang a lock or ready wait.

New topics under `/gimbalrotor/perching/multilink/`:

| Topic | Type | Meaning |
| --- | --- | --- |
| `pitch_rezero` | `std_msgs/Empty` | Start rezero from IDLE with a valid lock |
| `pitch_rezero_active` | `std_msgs/Bool` | ACTIVE only |
| `pitch_rezero_ready` | `std_msgs/Bool` | Successful HOLD |
| `pitch_rezero_failed` | `std_msgs/Bool` | Failure HOLD |
| `pitch_joint_hold_error` | `std_msgs/Float64` | Wrapped measured minus frozen pitch joint [rad]; zero in IDLE, NaN if unavailable |
| `secondary_joint_hold_error` | `std_msgs/Float64` | Wrapped measured minus frozen secondary joint [rad]; zero in IDLE, NaN if unavailable |
| `passive_pitch_delta` | `std_msgs/Float64` | Commanded passive rotation [rad] |
| `body_pitch` | `std_msgs/Float64` | Measured BASELINK world pitch [rad] |
| `body_pitch_rate` | `std_msgs/Float64` | Estimated BASELINK pitch rate [rad/s] |

`target_body_pose` continues to describe the desired BASELINK pose in both phases.
All new navigator defaults are documented in `MultilinkPerching.yaml`.
