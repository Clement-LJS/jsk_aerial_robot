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

Manual motion uses `gimbalrotor/RollYawMotion` on
`/gimbalrotor/perching/roll_yaw/motion`. Modes `ALPHA`, `ROLL`, and `YAW` are absolute radians in the
lock-relative passive coordinate or calibrated URDF servo coordinates. They are
**not body Euler angles**. Each changes only its own accepted destination; quick
axis changes retain the others. Effective configured/URDF bounds are published as
`position_lower`/`position_upper`. Nominal and applied feedback (`nominal_angles`,
`target_angles`) use `Vector3Stamped` wire order **[roll, alpha, yaw]**. Internal
kinematics use **[alpha, roll, yaw]**. The keyboard clamps its accumulator before
sending, so reversing a key at a limit immediately changes the destination.

Absolute mechanism destinations use a persistent online bounded follower. Each
axis retains its accepted destination independently of the applied/measured state.
Identical absolute refreshes do not restart motion. Each actual ROS-time interval
is integrated with bounded substeps, trapezoidal positions and acceleration-limited
rates, including a reserve for the final discrete braking step. A reversal retains
the replacement destination while braking and reverses without a second keypress.
The follower uses half the configured joint rate/acceleration budgets, reducing
acceleration further when the CoG/braking checks require it. Acceleration is bounded;
this online route does not impose a jerk limit or promise continuous acceleration
across target changes.

Endpoint admission checks FK/attitude and effective position limits. Before feed,
updates check the interval and a finite stopping corridor against joint, attitude
and changing-CoG derivative limits. Nonlinear corridor validation is sampled, not
a continuous feasibility proof. Failure to feed uses bounded braking with the
destination retained; inability to produce valid braking/holding remains a fault.
The full update commits one body/CoG/active-joint reference; rejected updates earn
no hidden displacement. Passive alpha is never published as an actuator command.
Automatic alignment and cut-start connections retain offline path validation.

Each request carries the UInt64 contact `session`, the published `motion_epoch`,
a unique client `source`, and an increasing `sequence`. One source owns motion
until stop, pause, disable, relock or setup takeover revokes that epoch. Stop does
not recapture the contact. Old epochs and reordered requests are rejected before
they can replace newer pending targets. `scripts/roll_yaw_motion.py` implements
this protocol for both supplied clients. Rebuild the package after updating it
so the generated message and navigator agree; then restart/source the workspace.
Press space to release keyboard ownership before starting the automatic script.
The script pins its epoch while waiting, so a later operator stop abandons that
waiting task too; restart the script deliberately to begin a new task.

**Latest destination:** pending updates to the same axis replace each other;
different mechanism axes retain their destinations. Body-pitch and velocity modes
replace incompatible pending motion. Input never waits for the previous movement
to finish. Lifecycle requests retain priority; observations have separate slots.
The bounded follower still brakes before reversal/takeover as necessary.

Keyboard `o/u` now changes one total `BODY_PITCH` destination: +0.2°, +0.4°,
+0.6°, then a negative key gives +0.4°. Its `frame` identifies the body-local +Y
starting orientation, captured after any preceding mode brakes. Switching modes
creates a new frame, even when intermediate inputs coalesce within one update.
`i/k` remains passive alpha. Defaults are `~body_pitch_step_deg:=0.2`,
`~body_pitch_rate_deg_s:=0.5`, and `~body_pitch_limit_deg:=20`.
The navigator approaches the destination through the existing bounded local-Y
velocity generator, slowing near the endpoint. Releasing a key retains the target;
space cancels it and brakes to hold. This is target control, not key-release stop.

The optional `JOG` mode retains small relative endpoint-IK increments (at most 1°).
A busy jog is rejected promptly, never retained for delayed execution. It is not
used by the keyboard. Old unscoped `Float64` motion inputs (`alpha_target`,
`roll_target`, `yaw_target`, `body_pitch_delta`, nonzero `body_pitch_velocity`),
the old vector target, and roll-yaw manual-pitch/nav shortcuts reject motion with
a migration acknowledgement. An unscoped zero velocity remains an operator stop.
Nojoint and active-pitch multilink inputs are unchanged. Numeric body-alignment
and prepared-cut lifecycle interfaces below are also unchanged.

`command_ack` distinguishes received/`queued`, accepted/`waiting`, `moving`, `replaced`,
`reference_complete`, `cancelled`,
`failed`, `rejected` or `noop`. Reference completion means the commanded endpoint
was reached; measured pitch, endpoint error and `measured_goal_reached` are
reported separately (also continuously in diagnostics). The
keyboard reconciles accepted nominal feedback before later joint steps.

Space brakes then cancels. Temporary invalid feedback or contact residuals hold
the last applied reference and retain the destination/contact identity. Both required
body and joint source stamps must advance, remain fresh/synchronized and pass frame,
array and contact checks before recovery; no per-increment settling gate is added.
True observation loss zeros derivatives once as an emergency hold, without claiming
acceleration continuity. Manual tracking then restarts the bounded follower, whose
progress does not depend on a whole-distance polynomial duration. Finite endpoint
jogs resume the same follower from the held pose, without reconstructing a path. Requests rejected during a hold do not become accepted later.

Measured mechanism tracking, reachable-to-measured body rotation residual, raw
commanded-to-measured body attitude error and independent pivot error are separate.
Manual/jog/velocity feed uses a memoryless slowdown above `soft_tracking_fraction`
of the existing hard tracking limits, plus candidate and stopping-reserve checks.
There is no 0.012 rad release latch. A steady 0.020 rad raw residual permits a
bounded step when the remaining envelope permits it; it is not subtracted away.
Axis lead reduces its own feed; raw body limits independently protect all motion. Distance to the accepted future endpoint is not a
tracking fault. Prepared trajectories retain progress and measured-goal completion
checks; reference completion alone cannot report physical success. Explicit pause,
cancel, stop and producer takeover override automatic recovery, including a pause
received while feedback is missing. A persistently blocked goal remains pending
with its reason/timeout status. `feedback_waiting` identifies observation/clock holds;
`tracking_hold` identifies the measured tracking governor. Diagnostic `hold_reason`
values are 0=none, 1=feedback, 2=contact, 3=clock, 4=tracking, 5=feasibility.
Contact residuals that remain inconsistent stay held;
recovery never recaptures the pivot to hide them. Physical operating-limit violations,
invalid topology/inertias/session identity and invalid holding outputs retain faults.

Timing parameters under `navigation/roll_yaw_perching` (seconds, also inherited by
simulation):

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `max_execution_step` | 0.05 | Manual/velocity step cap; prepared-path substep bound |
| `max_execution_gap` | 0.15 | Outage threshold; manual/velocity discard elapsed time above the step cap |
| `observer_max_interval` | 0.15 | Separate sample-interval limit for the existing implicit momentum observer |
| `feedback_timeout` | 0.15 | Required source/receipt freshness; unchanged |
| `feedback_sync_tolerance` | 0.05 | Required sample synchronization; unchanged |

Manual, jog and velocity updates integrate at most `max_execution_step` once;
extra elapsed time is discarded. An interval exceeding `max_execution_gap` holds
until advancing coherent samples return. There is no catch-up jump. Prepared
paths retain their existing bounded subdivision policy. The observer retains its
independent sample-interval limit; observer bootstrap or invalidity cannot gate
manual position navigation. A real observer rebase still invalidates tare.

Zero elapsed ROS time does not advance motion. Backward time holds the reference,
resets affected histories and expires the old velocity lease; a new valid source
epoch is required to resume finite motion. `/clock` pause does not accrue motion.
Steady receipt-age checks remain independent of source ROS timestamps.

The reference evaluator caches three rigid mass groups separated by the actual
KDL roll/yaw edges at capture. Fixed transforms, inertial origins, captured internal
joint positions and attached extra masses are folded into each group's mass and
first moment. Each evaluation computes two active joint poses and the exact
mass-weighted articulated CoG; it retains the captured desired CoG orientation.
Body/contact FK and the angular Jacobian use short, allocation-free chain passes.
Body/CoG directional derivatives use exact transform products, inverse derivatives
and mass-weighted first/second derivatives. Generic library callbacks retain their
finite-difference fallback; the captured roll–yaw model uses the analytic route.
No rounded-angle cache or frozen lock-posture CoG is used.

Feedback ingestion has its own short mutex. Body/joint callbacks validate and copy
inputs there; joint rates are differentiated from consecutive advancing **source**
stamps at the producer rate, with producer identity retained. The update owner
copies that state, then runs reference/observer work without the ingestion mutex.
Motion uses replaceable pending destinations. One command is admitted per update;
relative jogs reject busy state rather than becoming an ordering barrier.
Acceptance and request-specific acknowledgements occur on the update owner. An ordinary
absolute refresh can wait for that update without invalidating an in-flight
reference. Stop, pause, disable and relock mark the current
calculation obsolete. Commit rechecks physical eligibility and installs body/CoG
and joint output together; unpublished progress is discarded on preemption.
A stop during feedback loss clears the owner before automatic recovery. The
controller/session mutex still serializes controller readers and common navigation,
but it is not acquired by roll/yaw feedback or command-ingestion callbacks.

Reference-kernel bounds (excluding contact/diagnostics and observer ingestion):

| Kernel | Maximum synchronous work per update |
| --- | --- |
| Manual or admitted finite jog | 4 attempts × 2 analytic references, plus 2 for braking: 10 short-chain/CoG evaluations; zero IK |
| Local-Y velocity | 4 attempts × one 8-iteration IK, two derivative evaluations and one endpoint validation, plus 2 for braking: at most 14 CoG evaluations and 83 body-chain/Jacobian passes |
| Finite jog admission | Zero synchronous IK; one worker solve, at most 16 iterations |
| Observer dynamics | Zero synchronous dynamics evaluations; at most one outstanding worker job |

Ordinary successful manual feed needs two analytic reference evaluations. A
successful velocity attempt normally needs fewer than eight IK iterations, but
timing must be measured. Contact checks, diagnostics and observer input validation
add bounded short-chain evaluations; the counters include them. Allowing three
contact evaluations (snapshot, newer commit samples, then diagnostic snapshot),
one endpoint admission and one observer input reconstruction gives conservative
whole-update bounds of 15 CoG / 23 body-chain passes for manual motion, and
19 CoG / 96 body-chain passes for velocity. Ordinary successful updates use much
less. These bounds exclude capture, explicit rezero and prepared-path operation.
Model-signature
checks still scale with fixed topology size, twice per update, without tree FK.
No manual loop scales with endpoint distance, lease duration, missed periods or
keypress backlog. Prepared offline planning remains on its existing worker.

The online stopping certificate uses independent axis deceleration, with travel
`v_i² / (2 a_i) + |v_i| h / 2`. Rotation travel is bounded by triangle inequalities;
CoG speed/acceleration by `L S` and `L (A + S²)`, where L bounds all fixed offsets
and S/A weight the revolute rates/accelerations in the product of transforms.
Weights `[1,3,3]` conservatively include the hinge, inverse contact chain and
mass-group/body transforms. Per-update Bernstein derivative hulls bound the
entire connecting interval, including coordinate reversals; endpoint samples
are not used as a proof. CoG tilt <= 1 rad is a sufficient conservative certificate
for the existing firmware Euler bounds, not a widened physical limit. Conservative
bounds can slow or hold otherwise feasible postures and report that reason.

The observer's immutable-model worker returns source timestamps, Jacobian and
momentum terms. The owner applies the existing momentum update only to current
session/epoch results within source and wall-time freshness limits. Invalid or
stale worker output never authorizes admittance. Observation continues independently
of prepare/tare and position ownership. The source callbacks remain short.

Model validation now reports the first mismatch with category, named field,
captured/current values and numeric difference. It distinguishes temporary model
unavailability (recoverable hold) from a fixed topology/geometry/inertia/module
change (fault). It never compares live joint/gimbal angles, evolving FK, world
pose or articulated CoG. Floating inertia comparisons use relative/absolute
1e-12 numerical tolerance. Fixed joint origins, rotations and axes are captured
from URDF scalar fields. The calculation tree is rebuilt from that URDF, and
contact capture copies joint positions under the robot-model update mutex.
The checker must not call `pose()` or `getFrameToTip()` on the live articulated
KDL tree: those const methods use mutable joint caches while FK is running.
Run 4 recorded false changes in `gimbal_link1.joint_zero` and `thrust1.tip`,
including exactly `cos(0.5)` from the checker's own half-radian probe. Removing
these live probes prevents normal FK from invalidating the contact session.
Real URDF, topology, inertia and fixed payload changes still require recapture.

**Configuration synchronization limitation:** the base RobotModel outside this
folder has nonvirtual add/remove/set-extra-module methods and const-reference map
getters, with no shared configuration lock or generation. This repair captures
immutable calculation models and detects sequential fixed changes before use and
commit, but cannot make concurrent external map mutation safe. Configuration
writers must be quiescent while attached. A transactional writer/read-snapshot API
requires edits in `aerial_robot_model`, outside the authorized folder; local mutexes
or hidden derived setters cannot cover those base-class calls.

Existing `diagnostics_status` now includes `completed_update_sequence`,
`completed_update_stamp_s`, `navigation_wall_s`, `completed_*_wall_s`, input and
session lock wait/held durations, subdivisions, fast-model/body-chain/dynamics-tree
counts, and `navigation_over_25ms_count`. These timings/counts describe the
**previous completed update**, as labeled by `timing_scope`; nonlinear validation
time is included within reference time. `observer_worker_wall_s` and
`observer_worker_tree_calls` separately describe the latest completed worker
sample; they are not added to synchronous reference work. Current snapshot/commit validity, hold
reason, pending destination, axis tracking errors, braking reserves, source/receipt
ages, callback lock wait, joint callback queue age, coalesced joint samples and
joint-command publication period distinguish execution from last-command status.
Transport queue loss is not measured and is labeled separately. Large diagnostics
remain throttled to 5 Hz, outside the ingestion lock; command corrections use small
immediate messages. State/session lock duration may cover the calculation while
input lock duration should remain short.

The configured main loop is 40 Hz and still invokes navigation then control on the
same thread. Onboard acceptance remains: ordinary reference work of a few
milliseconds, typical navigation near/below 10 ms, regular publications near
25 ms, and sustained motion/reversal in one session. These are validation targets,
not measured performance results. The November 16 local repair starts at
`a93f9555` and keeps its consolidated navigator, controller dispatch, PID gains,
bounded follower and continuous tracking slowdown. The historical active-pitch
implementations (`2a59619b`, September/October successful cuts) and roll–yaw
implementation (`aa867f4c`, with later `b3ecbc5c` tuning) were compared with the
restructured lineage. `261106/15_cut_success_manual.bag` completed manual motion
but also recorded eight hinge-residual faults and repeated manual resets; it is
not evidence that the old roll–yaw implementation could finish unattended.

The four `261115` recordings were matched to source using rosout file/line numbers
and local Git history/reflog. This is source attribution, not an embedded binary SHA:

| Recording | Matching source | Recorded failure and regression coverage |
| --- | --- | --- |
| `1_afterperch_navfailed.bag` | `535e16d6` | 1.51 s processed-joint gap; hold without integrating missing time, preserve goal/session, resume after fresh feedback |
| `2_aftercodex_keyboardnavfailed.bag` | `84dcd6d5` (before amend) | Up to 343 ms callback and 365 ms navigation time; preserve the latest bounded work and freshness checks |
| `3_navfailed.bag` | `7d810631` | About 0.02059 rad residual trapped the old 0.012 rad release latch; retain continuous slowdown, test stop/recovery to the original yaw goal; accepted commands cannot count as completed motion |
| `4stillfailed.bag` | `a93f9555` | Keyboard and velocity motion progressed, then false fixed-geometry faults forced resets; use immutable URDF geometry instead of live KDL cache probes |

The read-only bag replay checks 1,808 active-contact samples against the compiled
feedback/governor helpers, and checks that the automatic task cannot report those
failed timelines as successful. Separate execution tests cover recovery and measured
completion with an ideal follower, including the older 0.0309128 rad hinge excursion.
The script tests cover a 1.51 s outage, completion after recovery, both pitch signs,
duplicate measurements, faults, session changes and absent measured progress.
Run 5 adds 1,322 recorded keyboard inputs and 52 velocity inputs to a replacement
policy replay at 30 Hz and 5 Hz. It checks that at most one body/velocity destination
is pending. Separate tests cover cancellation generations, rapid reversal, body
frame changes, Ctrl-C before ROS shutdown, and stop acknowledgement loss.
Build and offline checks use temporary artifacts and start no ROS master or flight
nodes:

```bash
bash test/run_roll_yaw_checks.sh
# Substitute the build directory printed by that command:
PYTHONDONTWRITEBYTECODE=1 python3 test/replay_perching_bags.py \
  --bag-root /home/clement/rosbag/year2026/11Nov/261115 \
  --core-test /tmp/gimbalrotor-checks.XXXXXX/devel/lib/gimbalrotor/roll_yaw_core_test
```

Input replay is not a closed-loop simulation of the modified aircraft. Raw servo
messages also contain malformed packets (30 in run 1, two in run 4); their upstream
transport source is unchanged. Temporary observation loss remains recoverable, but
persistent corruption, contact residuals, limits or true model changes cannot be
treated as successful navigation. The physical 0.03 rad hinge and 0.01 m pivot
checks remain unchanged. Hardware validation is still required.

Optional preplanned cuts still use prepare and start. Manual commands need no
tare/admittance; taking ownership releases planned compliance without resetting a
physically valid raw observer. Disable/relock retain their explicit meanings.

Examples for an already-running robot stack (namespace defaults shown):

```bash
rosrun gimbalrotor keyboard_roll_yaw_perching.py _body_pitch_step_deg:=0.2
# e -> z -> wait ready-HOLD -> b. j/l yaw, a/f roll, i/k alpha.
# o/u changes the total body-local pitch destination; space cancels it.
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
last number is not indefinitely valid. Readiness expires at `feedback_timeout`
in both sample age and steady receipt age. The separate `observer_max_interval`
bounds advancing observer sample intervals; numerical navigation substeps do not
rebase a physically fresh observer.
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
0.05 s maximum numerical substep, the reserved half-rates give 0.00375 rad maximum
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
requests an explicit operator stop and hold. Nonzero operation publishes the
`VELOCITY` mode of `RollYawMotion` in **rad/s** on
`/gimbalrotor/perching/roll_yaw/motion`. A legacy
`~step_deg` parameter is an error. `~interval_sec` defaults to 8.0 positive finite
seconds; `~num_steps` defaults to 8 and must be a positive integer, not boolean.
These define a signed angle goal: rate × interval × count. For the example above,
the goal is +6.4°; 64 seconds is its nominal duration at full speed. Holds, bounded
acceleration and tracking slowdown extend elapsed time rather than shorten the goal.
One stroke must be smaller than 180° in magnitude.
`~robot_namespace` and `~debug_prefix` retain keyboard defaults and absolute-prefix
handling. The script reads the positive onboard command lease once; it does not
change parameters. `~refresh_period_sec` defaults to lease/4 (2 s for the 8 s YAML
lease), and must be at most half the lease. A monotonic wall clock drives refreshes
and timeouts; source ROS timestamps are checked for freshness. The script waits for
fresh contact diagnostics, reference and measured orientation, and any preceding
manual destination to finish, before capturing the stroke start. It slows near the
reference endpoint, sends zero, and requires the measured endpoint, stopped motion,
advancing measurement stamps over 0.3 s and an accepted stop acknowledgement before
reporting completion. A stationary measurement cannot complete a small stroke merely
because it lies inside the navigator's normal tolerance.

`~goal_tolerance_deg` defaults to the navigator's goal tolerance and is capped at
10% of the requested stroke. `~feedback_timeout_sec` defaults to 1 s;
`~progress_timeout_sec` defaults to 30 s without measured forward progress.
`~timeout_sec` defaults to the larger of three nominal durations or the nominal
duration plus twice the progress timeout. Temporary holds send zero and retain the
remaining goal and contact session; fresh feedback allows a new velocity request
without disabling/enabling perching. A fault, session change, timeout, backward ROS
time or interruption sends a scoped STOP and ends the task. SIGINT/SIGTERM are
handled before ROS shutdown, with bounded stop retries and confirmation of the
new epoch and stopped diagnostics. Operator cancellation revokes the task; it never
adopts a new epoch to resume itself. Scoped zero velocity is a temporary pause in
the same task. The finite lease remains a fallback for process/transport loss.
Rejection never renews it. No prepare/tare/enable is sent.

The initial state is the coherent applied `final_target_`, including derivatives,
with an existing producer braking before takeover when necessary. The contact is
not recaptured. Each reference rotation right-multiplies the current body frame:
`R_next = R_current * Exp(e_y * integral(u dt))`, so angular velocity is `[0,u,0]`
in the commanded main body and `R_wb * [0,u,0]` in world. Constrained IK and its
Jacobian derivative coordinate passive alpha, active roll and active yaw; fixed
contact FK supplies body/CoG poses and feedforwards through the existing controller.
This nominal command requires no admittance baseline and preserves raw observer
history on refresh/takeover. `updateInteraction` still returns early without a valid
prepared plan: this velocity-refresh script does **not** exercise prepared-path
admittance. It does not enable compliance or add a velocity/force-control law.

The navigator uses a small orientation Jacobian and bounded local endpoint IK
with accumulated orientation error checked against a retained ideal rotation.
Accepted rotation progress right-multiplies that ideal reference; rejected motion
never accumulates future angle. Singularity, joint derivative limits, CoG limits,
tracking and continuous stopping/interval bounds can reduce or stop feed. A
rejected extension uses the common mechanism brake; local-Y feed resumes from
its resulting coherent applied posture. Requested/applied rates, remaining lease,
blocker and endpoint errors are diagnostic quantities. Measured task completion is
separate from command acceptance and from the requested rate's nominal duration.

Temporary invalid feedback uses the existing emergency hold (derivatives cleared,
not an acceleration-continuous stop). Recovery resumes from the held posture only
within the original lifetime; missing time is never integrated. Invalid messages
do not renew it. Stop/cancel, manual ownership, disable, relock and genuine faults
prevent stale rate continuation onboard. The task retains its goal across temporary
feedback holds and refreshes only in the original valid contact session. It never
recaptures a changed session or clears a genuine fault to continue the experiment.
There is no direct actuator publication.

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
