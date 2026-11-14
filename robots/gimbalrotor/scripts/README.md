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
update independent destinations; body-local pitch keys update the nominal destination through endpoint IK.

| Key | Action |
| --- | --- |
| `r`, `t` | Arm, take off |
| `e`, `d` | Enable/capture, disable perching |
| `j`, `l` | Active yaw joint − / + |
| `a`, `f` | Active roll joint − / + |
| `i`, `k` | Passive-hinge alpha + / −, relative to the current lock |
| `o`, `u` | Relative body-local pitch down (+) / up (−), including during motion |
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

The separate `body_pitch_delta` (`std_msgs/Float64`) requests one small relative
rotation about the accepted nominal body +Y: `R_goal = R_nominal * Ry(delta)`.
Keyboard `o/u` sends positive/negative `~body_pitch_step_deg` (default 0.2°).
`i/k` remains passive alpha. The navigator's `body_pitch_jog_max_delta` defaults
to 1° in radians. Larger, nonfinite or unreachable increments are rejected;
zero is a no-op. Each publication applies one increment; do not repeat an
increment as if it were an absolute destination.

Admission uses endpoint IK and the existing bounded mechanism interpolation.
All three coordinates may change. New increments retain unfinished accepted
motion, while interpolation starts from the coherent applied position, velocity
and acceleration. Reversals and mechanism-key takeovers are accepted during
motion; if a smooth replacement needs braking, the navigator retains one nominal
destination and connects automatically after braking. No key queue, prepare,
per-key settling, tracking-error admission gate or saw-point assumption is used.
Intermediate joint interpolation approximates pure body-Y rotation; the endpoint
matches the requested SO(3) orientation. Generated references still obey physical
position, attitude, joint rate/acceleration and CoG rate/acceleration checks.

`command_ack` reports `moving`, `replaced`, `reference_complete`, `cancelled`,
`failed`, `rejected` or `noop`. Reference completion means the commanded endpoint
was reached; measured pitch and endpoint error are reported separately. The
keyboard reconciles accepted nominal feedback before later joint steps.

Space brakes then cancels. Temporary invalid feedback or geometric residuals
hold the accepted goal and resume automatically when usable measurements return.
Requests received during invalid feedback are rejected without changing that
goal. Disable/relock and genuine physical/model faults retain their explicit
meanings. Optional preplanned cuts still use prepare and start. Manual commands
need no tare/admittance; taking ownership releases planned compliance without
resetting a physically valid raw observer.

Examples for an already-running robot stack (namespace defaults shown):

```bash
rosrun gimbalrotor keyboard_roll_yaw_perching.py _body_pitch_step_deg:=0.2
# e -> z -> wait ready-HOLD -> b. j/l yaw, a/f roll, i/k alpha.
# +0.2 degrees, then -0.2 degrees after reference completion:
rostopic pub -1 /gimbalrotor/perching/roll_yaw/body_pitch_delta std_msgs/Float64 'data: 0.0034906585'
rostopic pub -1 /gimbalrotor/perching/roll_yaw/body_pitch_delta std_msgs/Float64 'data: -0.0034906585'
rostopic echo /gimbalrotor/perching/roll_yaw/command_ack
rosrun gimbalrotor roll_yaw_command.py stop
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
`plan_ready=true`; `motion_active=true` means the connection is moving, while
`phase=waiting` retains geometry until live readiness returns.
Wait for `READY: MOTION_COMPLETE` on `state` before sending the second axis.
A denied/failed transaction reports `plan_ready=false`; correct its reason and
resend that numeric target. It does not retain the failed axis value. A busy or
stopping request is rejected for retry, not queued. There is no public `align`
command to recover a denied automatic alignment.

For a cut, first finish either keyboard/mechanism or numeric body alignment.
`prepare` captures fresh measured body orientation R0 and active-joint feedback
at the existing locked pivot. Small measurement inconsistencies are reconciled
with numerical IK only within `preparation_tolerance` (0.03 rad). Numerical IK
convergence remains much tighter. Plan identity uses the lock/generation and
unchanged coherent applied reference, including held derivatives. The captured
measurement defines the cut frame; it is not compared with later noisy samples
using final-goal tolerance. A validated connection starts at the held command
and reaches the fixed reconciled cut start without a jump. If current quiet or
preparation tracking readiness is missing, the computed connection waits up to
`settle_timeout` without another worker call. Even a zero-distance connection
waits for measured endpoint settling before READY. READY/start use current
tracking against that fixed prepared reference and `preparation_tolerance`;
`goal_tolerance` remains the measured endpoint criterion. Start is explicit.
There is no default realignment to lock-neutral.
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

Temporary missing/stale feedback, derivative bootstrap, body/joint timestamp
skew and pivot/hinge residuals hold the last coherent reference. Rejected packets
never refresh accepted measurements; empty joint velocity arrays require two
advancing fresh samples. The timeout remains 0.15 s and synchronization tolerance
0.05 s. Nominal joint and body-pitch destinations and contact identity survive.
Recovery uses a bounded transition from the held reference without integrating
outage time or silently recapturing contact. Moving prepared paths also resume
automatically after observation recovery; an explicit pause still requires resume.
Pending preparation may need resubmission if its coherent start is lost; this
optional interface is not required for manual commands or raw observation.

Ordinary tracking deviations do not latch faults or reset the raw observer.
Prepared-path feeding brakes and holds when tracking/progress is inadequate and
recovers automatically. Physical position/tilt violations, changed topology or
inertias, invalid generated references and invalid trajectory ownership remain
hard control faults. Nonfinite/missing physical inputs never yield valid loads.
Persistent geometric disagreement keeps a specific invalid reason and the same
captured contact. Invalid-observation holds zero feedforwards immediately and do
not claim acceleration-continuous physical stopping. Explicit stop/cancel/disable
override pending movement. Optional rezero retains its setup limits and timeout.

Prepared plans are never reused across sessions. Completion requires measured
endpoint orientation and mechanism tracking plus the configured settling window,
not merely elapsed time. `task_pitch` plots nominal/measured/applied local pitch;
the final requested goal remains separate in diagnostics.

With `admittance/source: articulated_momentum` (the hardware configuration), a
successful contact lock starts the articulated momentum observer automatically.
The first coherent physical sample rebases it; subsequent advancing samples
estimate the external generalized loads using `p = M(q) * q_dot` and the existing
momentum residual. Estimation continues during contact hold, manual
alpha/roll/yaw commands, body-local pitch jogs, alignment, preparation, READY,
execution, pause and completion. No prepare, start, tare or admittance-enable
step is needed to inspect it:

```bash
rostopic echo /gimbalrotor/perching/roll_yaw/generalized_external_load
rostopic echo /gimbalrotor/perching/roll_yaw/generalized_external_load_valid
rostopic echo /gimbalrotor/perching/roll_yaw/diagnostics_status
```

Under the configured roll–yaw debug prefix (default
`/gimbalrotor/perching/roll_yaw`), `generalized_external_load` is a
`sensor_msgs/JointState` with names `[alpha, configured_roll_joint,
configured_yaw_joint]` (normally `[alpha, joint_roll, joint_yaw]`) and three
`effort` values in N·m. Position and velocity arrays are empty. These are loads
in generalized coordinates, not a Cartesian force/torque vector or isolated
saw force; unmodelled loads and model/input errors also contribute.
`generalized_external_load_valid` is a `std_msgs/Bool`, true only for a ready,
current, physically valid result in this lock. Validity publishes at navigation
cadence and promptly on invalidation, independently of plotting subscribers.
Numeric loads publish only once per finite, ready observation, with its actual
body sample timestamp. Bootstrap, stale/invalid input and unperched operation
publish false and leave a gap in the numeric series: no NaNs, invented zeros or
restamped old values. Consumers must check validity **and freshness**; a retained
last number is not indefinitely valid. Readiness expires at `max_execution_step`
in both sample age and steady time since the last coherent observation.
Optional `task_pitch` and `compliance` vectors publish only when every represented
field is available and finite. Structured diagnostics retain finite control state
and validity flags, omitting unavailable numeric values.

The exact UInt64 `/session` remains command metadata. Exclude it from PlotJuggler
numeric import, or accept precision loss only for display; never use a rounded
display value as a command ID. There is no controller plotting dependency.

Diagnostics expose the same validity, age, stamp, coordinate order, loads and
`articulated_status` rejection/bootstrap reason.

The observer runs once in `applyRollYawTarget`, before control early returns and
path consumers, with its own physical feedback/contact and model guards. It
retains the HOVER/attachment,
timestamp, synchronization, inertia, finite-input and state-reconstruction
checks. ServoBridge current-derived joint efforts are consumed directly without
another calibration or sign/scale conversion.

The three-coordinate reduced model explicitly neglects internal rotor-gimbal
motion and its coupling dynamics. All represented masses and inertias, including
motor/gimbal assemblies, remain included at the internal configuration captured
at lock. Mass matrix, gravity, link geometry/Jacobians and predicted momentum
rate use that same reference with current alpha/roll/yaw. Physical gimbals may
move normally and remain at new angles without invalidating, rebasing or
requiring relock/tare. No gimbal velocity, derived rate, fresh gimbal timestamp
or comparison against the reference angle is required by the reduced observer.
Hand/lock/gimbal-only messages do not reset or refresh articulation feedback,
including messages from other publishers. The authoritative transformable-model
`getLinkJointNames()` / `getLinkJointIndices()` correspondence selects configured
roll/yaw inputs by name, independent of message or KDL order. Alpha is reconstructed
from the fixed-contact geometry and measured body motion, with no pitch servo
feedback or passive actuation torque. Other real link articulations are rejected
explicitly at capture rather than silently frozen.

The captured internal geometry is a modelling approximation, never a constraint
on live actuator angles. There is no live CoG drift comparison that can reset the
session when gimbals move. Topology, joint mapping, fixed link geometry,
mass/inertia and extra-module changes remain checked. Command speed caps constrain
references, not externally imposed measured rates; the constrained angular-rate
residual remains a physical consistency check.

The known actuation input remains the current world-frame command wrench about
the instantaneous CoG. Each accepted input retains the current body-to-CoG
offset from the live model. The generalized-force mapping shifts its moment to
body origin B exactly once using `moment_B = moment_CoG + (position_CoG -
position_B) cross force`. This current application point is separate from the
fixed internal inertial geometry; no live gimbal inertial terms are mixed into
the reduced momentum model. Omitted dynamics and actuation/model errors can
appear in the generalized residual: this is not exact Cartesian contact-wrench
recovery. Missing/stale/nonfinite required body, roll/yaw or actuation inputs,
or genuine contact/model disagreement, still produce unavailable output. The
three generalized torques cannot uniquely determine six independent wrench
components: with a matching body-origin Jacobian, `Q_ext = J_O^T w_O` is a
3-by-6 mapping. No external force contact point is assumed for the raw estimate.

Path/compliance resets preserve raw history and known-input caches. Disabling
perching, relock, source/configuration changes, actual model/coordinate recapture and
physical observation faults invalidate/rebase the observer. Rezero currently
recaptures the coordinate model at admission, so it rebases once there and then
observes throughout ACTIVE/HOLD while physically valid. Tare with unchanged
source/model/lock preserves raw history. Physical faults clear the compliance
baseline and cancel pending tare; raw recovery cannot re-arm compliance or
restore that baseline. Explicit disarm/tare/enable is required after such a fault.

For a prepared cut, the same timestamped residual is projected with its sampled
Jacobian: `J * tangent = direction * UnitY`, then
`task_torque = tangent.dot(generalized_external_load)`. `task_torque_valid`,
`task_torque_Nm` and `compliance.vector.z` remain signed trajectory-projection
outputs and may be unavailable before a cut exists. Missing/invalid paths or
projection failures do not erase a valid raw estimate.

Optional admittance uses this sequence:
**align → prepare → measured READY → explicit no-cut tare → baseline ready →
enable → start**. Ordinary cutting with admittance disabled needs no wrench
or tare.

When `admittance/source: legacy_wrench` is explicitly selected, the existing
wrench projection behavior is retained and the raw articulated output remains
invalid; legacy values are never relabelled as generalized residuals.
The built-in `estimated_external_wrench` is a
`geometry_msgs/WrenchStamped`: force in world coordinates (N), moment in world
coordinates about the CoG (N·m), timestamp from the IMU. The software convention
is wrench ON the robot: `wrench_scale: 1`, `wrench_sign: +1`. These conventions
are not physical calibration. This momentum observer uses model mass/inertia
and commanded input; perch reaction, model error and actuator error contaminate
it. `command_wrench_estimate`, pivot reaction alone and tracking error are not
isolated saw-force inputs.

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
**no cutting contact**; it is never automatic during a cut. Quiet-window failure
restarts collection. Articulated physical/source faults cancel tare and require
a fresh request after recovery; legacy source gaps retain their existing collection behavior. A new
preparation, lock, stopped/cancelled motion or changed tare convention clears
the baseline and disables admittance. Prepare therefore resets the previous cut's
compliance state. Enabling requires a READY cut, valid source configuration,
completed baseline and a fresh synchronized projected source.

Enabled admittance uses resisting external work along the geometric forward
stroke tangent, independent of the current velocity sign. Assisting work cannot
increase feed. Resistance freezes nominal feed and drives a bounded negative
stroke-angle offset. The executor brakes through zero and can retreat along the
same prepared path, within stroke endpoints, relief angle/rate/acceleration and
current measured tracking bounds. This reduces commanded penetration; physical
unloading of a jammed blade or calibrated force regulation is not established.

Body, articulated CoG and both active joints share the accepted signed phase.
Velocity scales with phase speed; acceleration retains both squared-speed and
phase-acceleration terms, including at a zero crossing. `offset_rad` is requested
relief; `effective_offset_rad` reports applied phase relative to the nominal
schedule. Current measured progress, separately from its historical maximum,
limits forward and reverse lead. Clearing resistance or disabling admittance
releases the correction with bounded motion, without changing the original goal.

Enabled stale/invalid interaction input brakes feeding and retains the goal.
Feeding recovers when the required input/baseline is usable, without an automatic
resume command. A raw-observer rebase invalidates the optional compliance baseline;
re-arming compliance still requires an explicit no-cut tare, never a silent bias
reset. Tracking protection remains active with admittance disabled. Lack of net
measured forward progress is reported without faulting or discarding the goal.
An explicit PAUSED state freezes progress and scalar integration while raw
observation continues; disabling compliance does not undo an explicit pause.
Stop/cancel and manual takeover release compliance; feedback loss during
cancellation completes cleanup at the held reference.
Relock/reset/fault invalidates the associated tare and correction.
The roll–yaw launch selects the ordinary `gimbalrotor_controller`, so the older
perching-admittance controller cannot inject a second correction.

Settings are read at these points (all under
`/gimbalrotor/navigation/roll_yaw_perching` unless stated otherwise):

| When | Settings/data |
| --- | --- |
| Initialization | Geometry, topics/frames, mechanism/attitude/feedback/governor/settling limits, planning bounds/deadline, rezero settings, admittance dynamics/contact gains and sample count |
| Enable/relock | Immutable URDF/inertia/extra-module snapshot, mechanism-limit intersection and captured contact; payload/geometry changes require recapture |
| Prepare / numeric body target | `request/*` for cuts is read outside the navigation mutex; model, measured/applied start and generation are captured together under the state lock |
| Explicit tare | Source/observer/effort settings and legacy `wrench_frame`, `wrench_point`, `wrench_sign`, `wrench_scale`; discards old baseline and requires new samples; unchanged articulated source preserves raw history |
| Start / resume | Fresh body/joint/quiet/start checks and enabled interaction freshness; no sampled path re-solve |
| Every active update | Feedback/contact/tracking guards, source age/synchronization, applied progress; held PLANNING/READY time is not integrated as motion |

Common blocking reasons and recovery:

| Reported reason | Ordinary recovery |
| --- | --- |
| Missing/stale session, HOVER/contact/feedback invalid | Restore coherent feedback/contact; recapture after a fault; helper rereads integer session |
| Quiet window incomplete, moving/stopping or worker busy | Wait for status/completion; resend (no queued commands) |
| IK/limits/singularity/attitude/CoG/downward-proxy failure | Choose a reachable alignment/feed within existing bounds; prepare again; no automatic sign reversal |
| Prepared, waiting for live readiness | Hold; the same connection begins once readiness passes, or expires at `settle_timeout` |
| Applied start/session changed | Reprepare; old geometry is invalidated |
| Invalid admittance configuration/source, wrong frame/point, no baseline | Validate source/convention, explicitly tare at quiet no-cut READY, wait baseline, then enable |
| Tracking blockage or transient observation failure | Goal/contact retained; feeding resumes automatically once usable feedback returns |
| Physical position/tilt violation or changed topology/inertia | Resolve the concrete cause before explicit reset/recapture; do not weaken physical limits |

The prepared-path lead/soft/tracking allowances remain 0.018 / 0.024 / 0.04 rad;
tracking allowances do not define contact faults. At the
0.05 s maximum moving step, the reserved half-rates give 0.00375 rad maximum
single-joint travel and 0.00875 rad summed body-rotation bound, below the
0.016 rad soft-to-tracking margin. The governor reserves discrete braking
distance; an externally displaced tracking boundary triggers bounded braking,
not a claim that measured tracking error can always be kept within that margin. `manual_callback_wall_s` and `navigation_wall_s` report observed
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
the configured external-wrench source before physical operation.

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

### `auto_roll_yaw_body_pitch_experiment.py`

After the existing roll–yaw perching/contact setup, this script publishes a
signed **main-body local +Y angular velocity**. The body is the main robot body
carrying the four rotors. Positive/negative follow its commanded local +Y at the
current roll/yaw configuration; this does not imply globally downward/upward
translation. The request is neither world Euler-pitch rate, CoG pitch, nor
passive-alpha velocity. Keyboard angle commands and `perching/manual_pitch_delta`
(absolute lock-relative passive alpha) keep their existing meanings.

```bash
# Illustrative arithmetic only; 0.1 deg/s is not a hardware-tuned recommendation.
rosrun gimbalrotor auto_roll_yaw_body_pitch_experiment.py \
  _angular_velocity_deg_s:=0.1 _interval_sec:=8.0 _num_steps:=8
```

`~angular_velocity_deg_s` is required, finite and numeric (not boolean); zero
requests bounded braking and hold. The ROS payload is `std_msgs/Float64` in
**rad/s** on `/gimbalrotor/perching/roll_yaw/body_pitch_velocity`. A legacy
`~step_deg` parameter is an error. `~interval_sec` defaults to 8.0 positive finite
seconds; `~num_steps` defaults to 8 and must be a positive integer, not boolean.
It counts rate publications. `~robot_namespace` and `~debug_prefix` retain the
same defaults and absolute-prefix handling as the keyboard.

The script reads
`<robot_namespace>/navigation/roll_yaw_perching/body_pitch_velocity_command_duration`
once before publishing and requires it to match `~interval_sec`. The YAML sets
8.0 seconds, inherited by simulation. Change both settings together; the script
does not write navigator parameters. The navigator reads its duration at startup.
Both expiry and sleeps use ROS time, including simulated time.

After the one-second startup delay, the same rate is sent once at approximately
0, 8, 16, 24, 32, 40, 48 and 56 seconds relative to the first publication, with
eight sleeps including the last. Each accepted nonzero message replaces the
requested rate and renews its lifetime from onboard acceptance; it adds neither
speed nor future jobs. The navigator advances the reference between requests.
With valid inputs and no takeover, the last interval expires around 64 seconds,
followed by bounded braking. Transport/loop jitter can affect boundaries.

The initial state is the coherent applied `final_target_`, including derivatives,
with an existing producer braking before takeover when necessary. The contact is
not recaptured. Each reference rotation right-multiplies the current body frame:
`R_next = R_current * Exp(e_y * integral(u dt))`, so angular velocity is `[0,u,0]`
in the commanded main body and `R_wb * [0,u,0]` in world. Constrained IK and its
Jacobian derivative coordinate passive alpha, active roll and active yaw; fixed
contact FK supplies body/CoG poses and feedforwards through the existing controller.
This nominal command requires no admittance baseline and preserves raw observer
history on refresh/takeover.

The navigator uses continuous scalar rate/acceleration ramps and checks a short
braking continuation before each advance, retaining that continuation if a new
advance is infeasible. It checks joint, singularity, attitude and CoG bounds;
nonlinear corridor checks are sampled, not a continuous feasibility proof.
Conservative reserves can reduce achievable speed. Existing diagnostics report
requested/applied rad/s, remaining lifetime and the limiting reason. No rejected
advance accumulates a future angle. Reversals brake smoothly; equal refreshes
preserve the running reference. Actual displacement integrates the bounded rate,
including ramps, holds and limits: 0.1 deg/s for 64 seconds would be 6.4 degrees
only in the ideal constant-rate case. This outer PID reference is not a guarantee
of measured angular velocity or exact displacement.

Temporary invalid feedback uses the existing emergency hold (derivatives cleared,
not an acceleration-continuous stop). Recovery resumes from the held posture only
within the original lifetime; missing time is never integrated. Invalid messages
do not renew it. Stop/cancel, manual ownership, disable, relock and genuine faults
prevent stale rate continuation. Ctrl+C simply ends publications: the last accepted
request may run for its remaining lifetime, then brake. There is no hidden stop,
enable/relock/prepare protocol, feedback polling, retry or actuator publication.

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
