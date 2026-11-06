**Last updated:** 2026-11-06

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
rosrun gimbalrotor keyboard_roll_yaw_perching.py _body_pitch_step_deg:=0.2
```

Tap a paired motion key for a small step (default 0.2 degrees). Mechanism keys
update independent destinations; body-local pitch keys submit finite asynchronous jogs.

| Key | Action |
| --- | --- |
| `r`, `t` | Arm, take off |
| `e`, `d` | Enable/capture, disable perching |
| `j`, `l` | Active yaw joint − / + |
| `a`, `f` | Active roll joint − / + |
| `i`, `k` | Passive-hinge alpha + / −, relative to the current lock |
| `o`, `u` | One coordinated body-local pitch jog down (+) / up (−); wait for completion |
| `z` | Explicit body pitch rezero with measured roll/yaw joints frozen |
| `b` | Final relock after fresh rezero ready/not-active/not-failed feedback |
| Space | Brake all mechanism/body motion together, then hold contact; discard the cut |
| `g`, `h` | Land, HALT motors |
| `s`, `?` | Status, help |
| `q`, Ctrl+C | Request stop, retain contact hold, restore terminal and quit |

Supported setup: **physically perch → e → z → wait PITCH_REZERO_HOLD → b → navigate**.
Enable only captures measured contact; it does not start rezero. Manual motion is
available immediately after a valid lock if no setup operation is pending. Rezero
uses multilink's signed measured-feedback routine, with roll/yaw frozen:
`alpha_dot = command_sign * clamp(kp * wrap(target_pitch - measured_pitch), ±rate_limit)`.
The integrated alpha is bounded by passive travel. Body position and attitude rotate
together about the same captured physical pivot. Readiness requires measured pitch
error and measured pitch rate to remain within tolerance for the stable duration.
ACTIVE and HOLD block ordinary motion. Failures retain the last valid body reference
and frozen active joints, with the cause in diagnostics.

The mounting pipe has approximately pi yaw. At zero active joints, contact/pivot +Y
maps approximately to body −Y, so positive alpha reduces measured world body pitch.
`pitch_rezero_command_sign: -1.0` therefore gives negative feedback, as in multilink;
this sign is applied once to alpha rate, separately from servo polarity. Nonzero
roll/yaw use the actual transformed hinge axis. A local FK response check rejects
a sign/geometry combination that would increase the pitch error; there is no
analytic endpoint solver or Stroke-based rezero controller.

The navigator **inherits the base relock callback**, as multilink does: enabled +
HOVER → reset old setup/targets → capture current measured joints/body → publish a
new base lock identity. It adds no stationary-body, completed-motion, or planner-stage
gate. Invalid measurements reject capture with an explicit reason. General relock
is available on `perching/relock` (`Empty`), including while moving. The keyboard's
`b` is specifically the multilink final-setup operation: it requires fresh rezero
ready=true, active=false, failed=false. On setup failure, use `d` and resolve the
cause before retrying. These keyboard checks are distinct from general navigator
relock policy. Successful relock refreshes every nominal baseline without zeroing
the active servos.

Mechanism `Float64` topics under `/gimbalrotor/perching/roll_yaw` are
`alpha_target`, `roll_target`, and `yaw_target`. Values are absolute radians in the
lock-relative passive coordinate or calibrated URDF servo coordinates. They are
**not body Euler angles**. Each changes only its own accepted destination; quick
axis changes retain the others. Effective configured/URDF bounds are published as
`position_lower`/`position_upper`. Nominal and applied feedback (`nominal_angles`,
`target_angles`) use `Vector3Stamped` wire order **[roll, alpha, yaw]**. Internal
kinematics use **[alpha, roll, yaw]**. The keyboard clamps its accumulator before
sending, so reversing a key at a limit immediately changes the destination.

Accepted destinations now use a bounded C2 mechanism interpolation from the
applied reference. Its joint position/speed/acceleration bounds cover whole
polynomial intervals using analytic Bernstein bounds. Each mechanism key performs an
algebraic duration search using the applied position, velocity and acceleration:
the quadratic bound roots partition feasible durations, including reversal and
near-zero-displacement cases where longer durations are not always feasible.
It also performs an endpoint FK/attitude check; it does not sample a whole path or submit planning
work. Each update samples only the next reference, checks joint, attitude and CoG
rate/acceleration limits before publication, and shares that FK between body,
CoG and both servos. Automatic alignment and cut preparation (including the
cut-start connection) retain full sampled validation in the offline worker.
The navigator rejects a transition that cannot satisfy those bounds. Small steps
remain available without offline IK, a saw proxy, thrust assessment, or admittance.
Reference motion retains acceleration limits when reversing a moving joint; physical tracking must still be verified.

```bash
# Absolute servo yaw and passive alpha, radians (not body yaw/pitch):
rostopic pub -1 /gimbalrotor/perching/roll_yaw/yaw_target std_msgs/Float64 'data: -0.3'
rostopic pub -1 /gimbalrotor/perching/roll_yaw/alpha_target std_msgs/Float64 'data: 0.05'
```

The separate `body_pitch_delta` (`std_msgs/Float64`) input requests **one relative
angular increment in radians**, from the coherent held applied body orientation
at admission: `R_des(t) = R_start * Ry(theta(t))`. Keyboard `o` sends positive
`~body_pitch_step_deg`, and `u` sends negative; this parameter defaults to 0.2°
and must be finite, positive and at most 1°. `~pitch_step_deg` still controls
passive alpha (`i/k`). Positive body-local +Y rotation is downward in the normal
operating alignment; passive-alpha sign is different because of the mounting yaw.
All three mechanism coordinates can change together. Body and CoG translations
follow the fixed-pivot FK; this is neither a world-Z depth nor a world-Euler pitch command.

Each jog uses the existing offline alignment worker, full geometric/derivative
checks, tracking governor and measured settling. Both signs require meaningful
saw-origin displacement in the requested vertical direction and a sampled path
that does not reverse that direction. Incompatible alignment/extent is rejected
without flipping the angle. This checks the rigid saw-origin proxy, not physical
tooth clearance, contact force or collision. Ordinary prepared cuts remain downward-only.
Nonfinite increments or `abs(delta) >= pi` are rejected; zero is a reported no-op.
Existing joint, body/CoG, singularity, rate and acceleration limits also apply.

Jogs require fresh valid HOVER contact, idle setup, a held reference and the normal
quiet preparation window. **They are discrete one-shot commands.** A request while
manual motion, automatic motion, a paused path, stopping or preparation is busy
is rejected without cancelling that motion or retaining a future jog. Wait for
completion, or press Space and then resend. No separate prepare/start is needed.
`command_ack` reports `action=body_pitch_delta`, the session, acceptance and reason,
with phases `scheduled`, `moving`, `failed`, `cancelled`, `rejected`, `noop` or
`complete`. Only `complete` means the measured endpoint settled; it includes the
requested delta and measured joint/orientation errors. The keyboard displays these
acknowledgements without blocking safety keys. `state` and diagnostics also report
preparation, execution, feedback pause and completion. The next jog is relative
to its new admitted start; no angular backlog is accumulated.

Space/stop/cancel invalidates pending planning or brakes the active path and holds
contact. If feedback disappears while braking, cleanup finishes at the already
applied pose with zero command derivatives, discarding the old goal and compliance
baseline. Observation loss during jog preparation cancels the job. During execution
it pauses with the original path retained; restore feedback and explicitly `resume`
or stop. Disable/relock also invalidate pending results. Completion leaves a hold,
not a prepared cutting stroke; `start` cannot repeat the jog. Subsequent body-yaw/
roll commands reconstruct their lock-relative alignment, and subsequent cut
preparation captures the newly measured orientation as R0.

Jogs are geometric manual navigation: they never enable or tare admittance, and
acceptance clears any old prepared-cut compliance state. For compliant cutting:
align/jog → prepare cut → measured READY → explicit unloaded tare → baseline ready
→ enable admittance → start. No wrench or tare is required for a geometric jog.

These examples are for an already-configured ROS environment with the normal
robot stack running (namespace defaults shown):

```bash
rosrun gimbalrotor keyboard_roll_yaw_perching.py _body_pitch_step_deg:=0.2
# e -> z -> wait ready-HOLD -> b. j/l yaw, a/f roll, i/k alpha.
# o: downward local-pitch jog; u: upward. Wait for completion. Space stops.
rostopic pub -1 /gimbalrotor/perching/roll_yaw/body_pitch_delta std_msgs/Float64 'data: 0.0872664626'
# After completion: -5 degrees from the new admitted start.
rostopic pub -1 /gimbalrotor/perching/roll_yaw/body_pitch_delta std_msgs/Float64 'data: -0.0872664626'
rostopic echo /gimbalrotor/perching/roll_yaw/state
rostopic echo /gimbalrotor/perching/roll_yaw/command_ack
rosrun gimbalrotor roll_yaw_command.py stop
# Restore coherent feedback before resuming a paused jog:
rosrun gimbalrotor roll_yaw_command.py resume
```

Keyboard `~robot_namespace` defaults to `/gimbalrotor`; `~debug_prefix` defaults
to `perching/roll_yaw` relative to that namespace (an absolute prefix is also
accepted). Match the navigator's configured `debug_prefix` when overriding it.

Numeric **body alignment** uses two separate `std_msgs/Float64` topics:
`body_yaw_target` and `body_roll_target`. Let N be the measured controlled-body
orientation at the current enable/relock. The target is
`N * Rz(yaw) * Rx(roll) * Ry(beta)`; all rotations are right-handed and in radians.
These are absolute intrinsic Z-X-Y alignment values relative to N, not world ZYX
Euler angles or servo angles. On the first body command after mechanism navigation
or hold, the untouched values and beta are extracted from the current measured
orientation relative to N. Subsequent accepted body commands retain the other
accepted values, including beta. Relock establishes a new N. All three mechanism
coordinates may change to reach the body orientation; no extra actuator publisher
is created. A body roll of 30 degrees does not establish an exact blade-plane angle.

```bash
# Send one, wait for its asynchronous result, then send the other:
rostopic pub -1 /gimbalrotor/perching/roll_yaw/body_yaw_target std_msgs/Float64 'data: -0.4'
rostopic echo /gimbalrotor/perching/roll_yaw/state
rostopic pub -1 /gimbalrotor/perching/roll_yaw/body_roll_target std_msgs/Float64 'data: 0.5235987756'
```

These commands solve an offline orientation path and automatically run its
alignment portion. `command_ack` reports admission/rejection, not completed
planning or motion. `state` and `diagnostics_status` report the eventual result;
`READY` after body alignment means the measured endpoint has settled. Requests
received while a worker is busy are rejected, never queued for later execution.
Both numeric body alignment and the preparation connection require valid geometry
and fresh coherent body/joint feedback. Watch `command_ack`
(`std_msgs/String`) for `JOB_ACCEPTED`, then `action=prepare_result` with
`plan_ready=true` and `motion_active=true`; this still means motion is in progress.
Wait for `READY: MOTION_COMPLETE` on `state` before sending the second axis.
A denied/failed transaction reports `plan_ready=false`; correct its reason and
resend that numeric target. It does not retain the failed axis value. A busy or
stopping request is rejected for retry, not queued. There is no public `align`
command to recover a denied automatic alignment.

For a cut, first finish either keyboard/mechanism or numeric body alignment.
`prepare` captures fresh measured body orientation R0 and active-joint feedback
at the existing locked pivot. Small measurement inconsistencies are reconciled
with numerical IK only within `preparation_tolerance` (0.03 rad). Numerical IK
convergence remains much tighter. The worker compares later measurements with
the submission measurement using `goal_tolerance` (0.0087 rad), so an unchanged
accepted tracking offset is not treated as new movement. It also checks the same
lock/generation and unchanged applied reference. A validated connection begins
at the held command and reaches the reconciled cut start without a jump. Even a
zero-distance connection waits for measured settling before READY. The actual
measured READY posture is retained for start/invalidation. There is no default
realignment to lock-neutral.
The requested motion is **R0 * Ry(d)**: a signed body-local pitch increment,
not world Euler pitch, passive alpha, a powered pitch joint, or linear depth.
The fixed saw-link origin is resolved from the actual controlled body frame
(`fc` for beetle); it is only a downward-motion proxy, not an identified tooth
contact. Preparation rejects upward proxy travel and never reverses the request.
Curved proxy motion is allowed. Both active joints can move during feed.

```bash
rosparam set /gimbalrotor/navigation/roll_yaw_perching/request/pitch_increment 0.1745329252
rosparam set /gimbalrotor/navigation/roll_yaw_perching/request/pitch_rate 0.03
rosparam set /gimbalrotor/navigation/roll_yaw_perching/request/pitch_acceleration 0.02
rosrun gimbalrotor roll_yaw_command.py prepare
# Wait for measured READY and read command_ack / diagnostics_status.
# Optional no-cut tare / admittance enable goes HERE (see below).
rosrun gimbalrotor roll_yaw_command.py start
rosrun gimbalrotor roll_yaw_command.py pause
# Wait for PAUSED; restore source/clear blockage before explicit resume.
rosrun gimbalrotor roll_yaw_command.py resume
rosrun gimbalrotor roll_yaw_command.py stop
```

Automatic alignment and cutting use geometric preparation, coordinated path
limits and measured feedback guards. They do not predict thrust allocation,
battery-dependent capacity or current-gimbal gravity support. No additional
voltage sample or opt-in is required for trajectory approval. READY reports
measured geometric settling, not verified thrust feasibility.

Existing firmware mechanical-angle/raw-position clamps and motor PWM limits,
servo profiles and controller allocation remain unchanged. Downstream clipping
does not guarantee that the requested body wrench can be produced after clipping;
existing body/joint tracking guards still respond when execution cannot follow
its references.

`prepare`, `start`, `pause`, `resume`, `stop`, and `cancel` accept `std_msgs/UInt64`
on the matching topics under `/gimbalrotor/perching/roll_yaw`, containing the exact
current `/session` value. The script reads that ID as an integer and checks the
acknowledgement. `pause` decelerates on the existing path; `resume` continues it.
`stop`/`cancel` and Space brake on the same scalar path for body, CoG and joints,
then discard the plan and report `stop complete`. An already-held stop completes
immediately. They do not command zero thrust or zero joint angles.

Temporary missing/stale feedback, derivative bootstrap and body/joint timestamp
skew hold the last coherent reference with all command derivatives zero. Rejected
packets retain the accepted measurement and its original timestamp; empty joint
velocity arrays require two advancing fresh samples. The timeout remains 0.15 s
and synchronization tolerance 0.05 s. Manual destinations and contact identity
survive this wait: coherent contact/tracking checks precede a fresh bounded
transition from the held reference, without catching up elapsed outage time.
Stop/cancel/disable override pending movement. Moving automatic paths enter
`PAUSED`, retain the original goal, and require explicit `resume` after fresh
quiet/contact checks. Pending preparation is cancelled if its coherent start is
lost; READY still requires its original measured-start checks. Rezero never
counts the gap as settling and retains its original timeout. An observation
hold zeros feedforwards immediately; it is not an acceleration-continuous physical
stop. Actual geometry, hard tracking, bounds and clock faults remain latched.
A fault requires resolving the cause and recapturing contact. A lock change, accepted manual target, or
movement beyond `goal_tolerance` from measured READY (or loss of cut-start
geometry consistency) invalidates the cut. Manual takeover during automatic
motion first brakes: wait for stop complete and retry the key/target. Independent
axis destinations and immediate reversal at a limit are preserved.
Prepared plans are never reused across sessions. Completion requires measured
endpoint orientation and mechanism tracking plus the configured settling window,
not merely elapsed time. `task_pitch` plots nominal/measured/applied local pitch;
the final requested goal remains separate in diagnostics.

Optional admittance uses this sequence:
**align → prepare → measured READY → explicit no-cut tare → baseline ready →
enable → start**. Ordinary cutting with admittance disabled needs no wrench
or tare.

The configured built-in `estimated_external_wrench` is a
`geometry_msgs/WrenchStamped`: force in world coordinates (N), moment in world
coordinates about the CoG (N·m), timestamp from the IMU. The software convention
is wrench ON the robot: `wrench_scale: 1`, `wrench_sign: +1`. These conventions
are not physical calibration. This momentum observer uses model mass/inertia
and commanded input; perch reaction, model error and actuator error contaminate
it. `command_wrench_estimate`, pivot reaction alone and tracking error are not
isolated saw-force inputs. The articulated observer remains diagnostic only and is not scheduled by default.
Set `navigation/roll_yaw_perching/diagnostic_dynamics: true` only to opt in to
that workload. This setting does not disable the built-in external-wrench
estimator or affect path admittance.

The built-in source defaults are `external_wrench_topic: estimated_external_wrench`,
`wrench_frame: world`, `wrench_point: cog`, `wrench_sign: 1.0`, and
`wrench_scale: 1.0`. No calibration acknowledgement parameter is required.
For a custom source explicitly configure its frame (`world`/`body`), moment point
(`cog`/`body`/`world_origin`), sign (+1 or -1), positive finite SI scale and timing.
Restart if changing the subscribed topic; explicit tare rereads the numerical
conventions and discards the previous baseline. Invalid configuration, frame,
values, nonadvancing/stale timestamps or unsynchronized input cannot provide a
usable baseline or enable admittance.
The projection shifts the moment once and uses the signed articulated body
translation/rotation tangent, not the old fixed pivot-Y projection.

```bash
# After prepare reaches measured READY, physically remove all cutting contact:
rostopic pub -1 /gimbalrotor/perching/roll_yaw/admittance_tare std_msgs/Empty '{}'
rostopic echo /gimbalrotor/perching/roll_yaw/diagnostics_status
# Wait for baseline_ready=true (80 distinct fresh, synchronized, quiet samples).
rostopic pub -1 /gimbalrotor/perching/admittance_enable std_msgs/Bool 'data: true'
# Check admittance_enabled=true and its status, then:
rosrun gimbalrotor roll_yaw_command.py start
# Disable explicitly; correction recovers with bounded path speed/acceleration:
rostopic pub -1 /gimbalrotor/perching/admittance_enable std_msgs/Bool 'data: false'
```

Tare works while disabled and at measured READY for a prepared cut. It asserts
**no cutting contact**; it is never automatic during a cut. Quiet/contact failure
or source gaps restart collection. A new
preparation, lock, stopped/cancelled motion or changed tare convention clears
the baseline and disables admittance. Prepare therefore resets the previous cut's
compliance state. Enabling requires a READY cut, valid source configuration,
completed baseline and a fresh synchronized projected source.

This is constrained path admittance/feed relief: resistance produces a bounded
nonpositive correction that slows or holds forward penetration. It does not
actively retract or unload a blade. A request behind current progress causes
bounded braking/hold, not an achieved reverse displacement. Body, articulated
CoG and both active joints use the same effective phase and scaled derivatives.
`offset_rad` is internal requested relief; `effective_offset_rad` and
`effective_phase_s` describe the applied response, separately from
`nominal_phase_s`, measured progress and the original goal. Saturation and the
blockage governor cap demand backlog. Clearing resistance or disabling releases
smoothly within speed/acceleration and measured-lead bounds.

Enabled stale/invalid wrench input brakes/holds and retains the goal and
correction; it is not zero resistance. Recovery before `progress_timeout` (30 s)
may continue smoothly. Sustained blockage/input loss brakes to PAUSED; after
restoring the condition use explicit `resume`. Stop/cancel and manual takeover
clear correction after bounded stopping. Relock/reset clears it immediately.
The roll–yaw launch selects the ordinary `gimbalrotor_controller`, so the older
perching-admittance controller cannot inject a second correction.

Settings are read at these points (all under
`/gimbalrotor/navigation/roll_yaw_perching` unless stated otherwise):

| When | Settings/data |
| --- | --- |
| Initialization | Geometry, topics/frames, mechanism/attitude/feedback/governor/settling limits, planning bounds/deadline, rezero settings, admittance dynamics/contact gains and sample count |
| Enable/relock | Immutable URDF/inertia/extra-module snapshot, mechanism-limit intersection and captured contact; payload/geometry changes require recapture |
| Prepare / numeric body target | `request/*` for cuts is read outside the navigation mutex; model, measured/applied start and generation are captured together under the state lock |
| Explicit tare | `wrench_frame`, `wrench_point`, `wrench_sign`, `wrench_scale`; discards old baseline and requires new samples |
| Start / resume | Fresh body/joint/quiet/start checks and enabled interaction freshness; no sampled path re-solve |
| Every active update | Feedback/contact/tracking guards, source age/synchronization, applied progress; held PLANNING/READY time is not integrated as motion |

Common blocking reasons and recovery:

| Reported reason | Ordinary recovery |
| --- | --- |
| Missing/stale session, HOVER/contact/feedback invalid | Restore coherent feedback/contact; recapture after a fault; helper rereads integer session |
| Quiet window incomplete, moving/stopping or worker busy | Wait for status/completion; resend (no queued commands) |
| IK/limits/singularity/attitude/CoG/downward-proxy failure | Choose a reachable alignment/feed within existing bounds; prepare again; no automatic sign reversal |
| Measurement/applied start changed during prepare, prepared start moved | Hold/settle and reprepare; resend denied numeric axis |
| Invalid admittance configuration/source, wrong frame/point, no baseline | Validate source/convention, explicitly tare at quiet no-cut READY, wait baseline, then enable |
| Enabled wrench stale or tracking blockage | Restore source/remove blockage; explicit resume after PAUSED; goal remains pending |
| Hard tracking/contact/clock fault, settling timeout | Resolve the reported cause, recapture contact and prepare again; do not weaken hard limits |

The default lead/soft/hard hierarchy remains 0.018 / 0.024 / 0.04 rad. At the
0.05 s maximum moving step, the reserved half-rates give 0.00375 rad maximum
single-joint travel and 0.00875 rad summed body-rotation bound, below the
0.016 rad soft-to-hard margin. The governor additionally reserves discrete
braking distance. `manual_callback_wall_s` and `navigation_wall_s` report observed
costs, including mutex wait for manual callbacks and updates. Callback timing
covers admission, construction and diagnostics; it is published on the next
diagnostics cycle. Source checks and standalone timing do not establish
real-time guarantees.

The legacy full-vector `target` interface still takes wire order [roll, alpha,
yaw] and `header.frame_id=perching_lock_<session>`. Legacy `manual_pitch_delta`
and pitch-only `uav/nav` mean lock-relative alpha. All inputs require enabled
HOVER/contact and fresh coherent onboard pose/joint feedback. Keyboard freshness
uses local receipt time; remote timestamps identify locks, not clock offsets.
Only active roll/yaw are sent to `joints_ctrl`; body and articulated CoG references
flow through the existing flight controller. No saw command is sent.

No flight/cutting outcome or hardware calibration follows from the offline
checks. Commission the pivot, model assumptions, actuator tracking and
optional observer source before physical operation.

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
