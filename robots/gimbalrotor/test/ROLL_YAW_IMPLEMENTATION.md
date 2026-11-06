# Local roll–yaw QP, measured progress and pitch admittance

Implemented against the clean local checkout on `develop/pochitab_passiveperching/main`,
HEAD `b39dfeed04c47d82d11aa4645cf6bd9314418337`. All source changes are inside
`robots/gimbalrotor`. No branch/worktree changes, commits, pushes, remote operations,
dependency installations, deployment, ROS node launches or hardware operation were performed.

The accompanying task document prohibited tests/builds, while the user's direct
request explicitly required them. The direct request was followed: the existing
`/opt/ros/one/setup.bash` and workspace `devel/setup.bash` were sourced in child
shells, and isolated CMake build/devel/install destinations were placed under
`/tmp`. The existing workspace build/devel directories were only read. No
`ros_env` helper, catkin workspace configuration or flight/simulation launch was run.

**What was verified in the original sources.** The checked-out revision matches
the document. The original executor completed on elapsed path time plus rate
settling; it did not test the original measured pitch endpoint. Its CoG callback
constructed a KDL tree solver per evaluation. The original wrapper launch lacked
the advertised `qp_planning` and `online_planning` arguments. These gaps are addressed.
The existing hardware settling values, simulation overrides, geometry, PID gains,
FC allocation, feedback tolerances and URDF remain intact. The settling history now
has a 4096-sample capacity bound; exceeding it restarts qualification rather than
certifying an incomplete window.

The frame audit used the local navigation, cutting trajectory, QP, kinematics,
ordinary controller, legacy perching admittance controller, base momentum observer,
admittance core, servo bridge, model inertia aggregation, launch files and beetle
URDF/configuration. No code was imported from `motion_planning`; that reference
was not present in the local ROS tree and was not fetched.

**Goal, progress and reference ownership.** `Plan::goal` and `Plan::neutral` capture
the request and lock convention. Preparing a new plan keeps the topic arm state,
baseline and compliance separate and starts from the held *nominal* reference.
Preparation is rejected while an existing trajectory is active. It does not use
the compliant endpoint as the new request. Alignment, measured settling, READY
and explicit start are retained. Explicit cancel abandons the producer and holds
the applied pose; it never reports physical success.

For the body measurement, `R_pre = N Rz(yaw) Rx(roll)`. The Y twist of
`R_preᵀ R_WB_measured` is unwrapped continuously; a degenerate twist is rejected.
Raw pitch, filtered pitch and full swing/SO(3) residual are distinct. The maximum
of the conservative directed raw/filtered progress is retained, so noise or
temporary reversal cannot rewind execution. Goal checks always use raw measured
orientation, not this monotonic history. Nonzero starts and both directions are
covered by the tests.

The offline path is sampled using measured progress plus a finite lead. Phase
speed and acceleration are bounded, including braking distance before consuming
the lead. The lead is `0.06 rad` (3.438 degrees), the maximum compliance offset is
`0.0383972 rad` (2.2 degrees), and the explicit normal tracking allowance is
`0.01 rad` (0.573 degrees). Configuration must satisfy
`lead > maximum_offset + tracking_allowance`. The separate goal tolerance is
`0.0087 rad` (0.498 degrees). These are implementation starting values, not approved
hardware tuning. Existing tracking faults can stop a blocked robot before it uses
all the lead; those guards were not widened. Lag has a 30-second timeout.

`path_end_commanded` means the nominal path was exhausted. `GOAL_REACHED` requires
fresh measured pitch and full orientation within goal tolerance, endpoint joint
agreement, existing pivot/hinge checks, and measured settling. The hardware
bounded-window thresholds remain body RMS/current `0.05 rad/s`, joint RMS/current
`0.03 rad/s`, mean drift `0.02 rad/s`, duration `0.5 s`, timeout `10 s`. Duplicate
measurement stamps do not add settling time. Nominal +10 degrees, control +9 and
measured +9 remains goal-pending and eventually times out under persistent load.
No automatic nominal overshoot compensates for admittance.

The navigator remains the sole reference owner. It sends one coherent body,
articulated CoG and active roll/yaw reference; passive alpha is never a servo
command. Tracking faults compare measurements with this applied reference.
Progress/completion compare measurements with the original request. Hard faults
retain their first cause and hold the last valid applied pose.

**Offline/online work.** Offline preparation still validates the complete alignment
and cut at the existing planning step and speed, with a 120-second cooperative
wall-clock deadline. FK solvers and joint buffers are independently owned copies.
The optional online mode defaults to false. It schedules at most every 0.05 seconds
when a new segment is needed, using a 0.5-second horizon, 0.2-second phase lookahead
and 0.1-second result deadline. These values are scheduling limits, not measured
onboard performance or force bandwidth. An active splice is finished before it
is replaced, so refresh acceptance is generally slower than the scheduling ceiling.

Online work removes the actually applied correction from the measured orientation
to obtain a nominal measurement seed:
`R_seed = R_measured R_appliedᵀ R_nominal`. Perfect tracking of the compliant
reference therefore gives zero nominal seed error. Differential IK refreshes the
interior toward the same original path/goal. A smooth correction preserves exact
position, velocity and acceleration at both splice boundaries; the original
offline path and its metadata are not overwritten. Bernstein bounds check joint
position/rate/acceleration throughout each polynomial interval; dense samples check
orientation, singularity, body/FC attitude and articulated CoG limits. Existing
derivative reserves allow bounded phase stopping. Up to eight bounded correction
weights are tried; infeasible results are rejected without weakening limits.

One persistent worker handles online planning, one handles offline preparation,
and one handles articulated dynamics. Each has one job/result slot, worker-owned
model state, cooperative cancellation and nonblocking callback polling/submission.
Workers never access the live robot model or navigator. Joins occur only at object
shutdown. Session/generation, elapsed time and fresh effective tracking are checked
when accepting an online result. Executor feedback guards run again at activation;
an unactivated result older than 0.5 seconds is discarded. Rejected/late updates
leave the validated offline nominal path available. This remains kinematic IK,
without force optimization or obstacle detection.

**Observer equations and sources.** For each copied inertia entry, its tensor is
shifted to its own CoG, transformed by link FK, and included once. The live model's
inertia map already aggregates fixed children; summing the tree inertias again
would double-count them. Registered modules are added once. The saw's rigid inertia
is included when present in that model.

With generalized coordinates `eta = [alpha, roll, yaw]`, world link CoG positions
`x_i`, linear velocities `v_i`, angular velocities `w_i` and world inertias `I_i`:

```text
g_world = -robot_model.getGravity3d()       # (0,0,-9.80665), m/s²
T       = 1/2 sum(m_i v_i·v_i + w_i·I_i w_i)
U       = -sum(m_i g_world·x_i)
p_eta   = sum(Jv_iᵀ m_i v_i + Jw_iᵀ I_i w_i)
M_eta   = sum(m_i Jv_iᵀ Jv_i + Jw_iᵀ I_i Jw_i)
b_eta   = Q_known + partial(T)/partial(eta) - partial(U)/partial(eta)
n_theta = solve(J_omega_body, [0,1,0])
```

Central differences evaluate FK derivatives (`1e-5`) and energy gradients (`1e-4`).
This includes configuration-dependent inertia and Coriolis/centrifugal terms through
the energy/momentum formulation. Moving other joints contribute to link velocity
and generalized momentum when fresh velocity feedback, or two position samples,
are available. Missing other-joint rates are explicitly frozen/modelled and counted
in `load_source`; they are not labelled measured zeros. The existing CoG snapshot
drift guard remains in force.

Body pose/rate come from synchronized, fresh baselink odometry. Active positions,
rates and efforts are selected by name from `joint_states`; passive rate is projected
from body angular velocity after subtracting active-joint contributions. Joint/body
and command/body synchronization and the existing 0.15-second freshness limit apply.
Missing/nonfinite active effort prevents a computable observer update, but missing
calibration does not add an enable gate.

The ordinary controller publishes a *command-derived* resultant about the current
CoG, expressed in world axes:

```text
F_world       = R_WG m a_command_G
tau_G_world   = R_WG (I_G angular_acc_command_G + omega_G × I_G omega_G)
Q_known       = Jv_body_material_point_at_Gᵀ F_world
                + (R_WB J_omega_body)ᵀ tau_G_world
                + [0, effort_roll, effort_yaw]
```

The translation is the total existing PID result; no second gravity force is added.
The positive gravity vector stored in RobotModel is negated for gravitational
potential. The gyro term is added once when converting FC-mode acceleration to
the estimated physical input moment. The existing published net external-wrench
observer is untouched, including its world force/world moment-about-CoG semantics.

All rotor inputs are on the body branch in this mechanism. Their resultant can be
projected at an instantaneous body material point coinciding with G, with the moment
about G. Its Jacobian is **not** the derivative of the articulated assembly CoG:
that substitution would introduce incorrect active-joint virtual work. Individual
rotor thrust/reaction measurements are absent. The resultant above is an estimate
of realized actuation, and FC-side tracking/allocation errors remain disturbances.
Active servo efforts are added separately; they cannot be recovered from a six-axis
CoG wrench. The bridge already applies `torque_scale * angle_sgn`. In the current
Servo.yaml the group scale is 0.01 and yaw sign is -1; neither is applied again here.
Whether this calibrated load really represents output-joint Nm, including gearing
and motor losses, still needs a bench check.

The residual uses a backward-Euler version of the existing observer principle:

```text
r_k = K (p_k - p_initial - integral_previous - dt b_k) / (1 + K dt)
integral_k = integral_previous + dt (b_k + r_k)
tau_body_pitch = n_thetaᵀ r_k
```

`K` defaults to 10 s⁻¹; initialization records actual momentum, and gaps reinitialize
the observer. There is a one-second initialization/recovery interval before using
the torque for compliance. Dynamics jobs run at most at 20 Hz with a 0.1-second
cooperative deadline; source timestamps determine integration dt. The SVD condition
and solve residual are checked. Near neutral, the approximately pi mounting yaw
maps positive passive alpha toward negative body-local pitch. The tests verify the
Jacobian sign, gravity sign and `r_eta·eta_dot = tau_body_pitch*theta_dot` convention.
No extra arbitrary pitch torque sign is introduced.

Ideal support reactions have zero virtual work in the constrained coordinates.
Unmodelled hinge/gear friction, support motion, structural flexibility, servo load
calibration error, rotor/FC mismatch and spinning saw/rotor momentum can still enter
the residual. URDF simulation friction is not assumed to be calibrated hardware
friction. A scalar task torque does not uniquely identify a six-dimensional saw
wrench or isolate cutting from every other disturbance.

**Admittance lifecycle.** The only operator arm/disarm interface is the existing
`/gimbalrotor/perching/admittance_enable` Bool. Default is disarmed. Keep the saw and
contact unloaded during baseline collection: after observer initialization, 80
distinct valid samples are averaged only while locked/held, planning or READY,
with quiet measured rates and before the stroke has ever started. At the nominal
20 Hz ceiling this takes at least about five seconds including initialization.
Collection pauses during alignment. No sensor can certify that this operator-chosen
period is actually contact-free. Bias is frozen during the stroke, pause, completion
and topic release; a new lock resets it. Articulation-dependent bias is a limitation.

True requires lock/HOVER, a ready baseline and fresh computable input, with rejection
reasons reported. False removes forcing and releases the live offset smoothly.
Stale torque similarly removes forcing, with recovery initialization before reuse.
The scalar law is `0.40 delta_ddot + 5 delta_dot + 4.5 delta = gated_torque` by default,
with a 0.1-second torque filter, 0.04/0.02 Nm contact hysteresis and 0.1/0.2-second
dwell. Torque, angle, rate and acceleration limits are 0.18 Nm, 0.0383972 rad,
0.15 rad/s and 0.1 rad/s². The reference torque is configurable, default zero.
The virtual dynamics use bounded implicit integration, braking before the angle
limit and constrained local IK with at most seven acceleration/braking candidates.
Rejected candidates do not wind up the scalar state. If no candidate is feasible,
the existing hard-fault hold takes precedence. These legacy-derived values still
need robot-specific tuning.

`R_control = R_nominal Ry(delta)` is reconstructed through the full mechanism.
Body, joint and articulated CoG derivatives are generated consistently. Opposing
negative task torque produces a negative correction during a positive stroke.
Online planning is unnecessary for this local synthesis. The old single-pivot
admittance plugin is not selected, and no additional PID/motor/FC writer was added.

**Topics and recording.** Existing String `state` (latched), String `diagnostics`,
UInt64 `session`, Vector3Stamped `nominal_angles`/`target_angles`, and odometry
references/measurements retain their names and types. Angle wire order remains
`[roll, alpha, yaw]`. Status snapshots publish at at most 5 Hz, plus state transitions.
Existing feedback timestamps are retained; remote manual receipt/session handling
is unchanged.

| New topic under `/gimbalrotor/perching/roll_yaw` | Type and meaning |
|---|---|
| `diagnostics_status` | `diagnostic_msgs/DiagnosticArray`; publication stamp, stable keys with units, lock/session/plan/generation IDs, first fault, planning timing/intervals/cancellation, settling, sources/ages/skew/invalid counts, geometry, separate goal/nominal/measured/control fields, torque/bias/contact, online rejection/expiry and callback wall time |
| `task_pitch` | `geometry_msgs/Vector3Stamped`; x nominal task pitch, y raw measured task pitch, z compliant control pitch; all rad |
| `compliance` | `geometry_msgs/Vector3Stamped`; x offset rad, y offset rate rad/s, z estimated task torque Nm |
| `command_wrench_estimate` | `geometry_msgs/WrenchStamped`; source IMU stamp, world axes, estimated input force N and input moment Nm about current CoG; published by the ordinary FC-mode controller when subscribed |
| `command_ack` | `std_msgs/String`; action, exact UInt64 session, accepted/rejected flag and reason; acceptance acknowledges the command, not planning completion or physical success |

Numeric plot headers are publication time and `perching_lock_<ns>`, not new sensor
timestamps. Use `body_stamp_s`, `joint_stamp_s`, `torque_stamp_s`, source ages and
validity fields in `diagnostics_status`; unavailable plotted values are NaN.
`settling_valid` distinguishes complete statistics from unavailable values.
Prepare solve/validation wall fields are cumulative phase measurements; total
elapsed wall time includes all preparation work. Record `/rosout`, the original
joint/body sources and command input estimate, and retain the model/configuration
used for the bag.

The following are operator examples only; they were not executed:

```bash
roslaunch gimbalrotor roll_yaw_perching.launch simulation:=false qp_planning:=true online_planning:=true
# Perch, optionally rezero, and perform the existing final relock first.
rosrun gimbalrotor roll_yaw_command.py prepare
# Wait for validated plan readiness, then:
rosrun gimbalrotor roll_yaw_command.py align
# Wait for READY and baseline_ready; keep this baseline period unloaded.
rostopic pub -1 /gimbalrotor/perching/admittance_enable std_msgs/Bool 'data: true'
rosrun gimbalrotor roll_yaw_command.py start
# May be changed independently during the stroke:
rostopic pub -1 /gimbalrotor/perching/admittance_enable std_msgs/Bool 'data: false'
rostopic echo /gimbalrotor/perching/roll_yaw/state
rostopic echo /gimbalrotor/perching/roll_yaw/diagnostics
rostopic echo /gimbalrotor/perching/roll_yaw/diagnostics_status
rosbag record -a
```

The helper reads the current UInt64 session, validates serialization without a
float conversion, waits for a navigator subscriber, sends one command and waits
for a matching acknowledgement. It assumes one operator command source; the
existing UInt64 wire protocol has no per-command nonce. Change `--namespace` when
using a nondefault robot ID. No historical lock ID is hardcoded.

**Verification actually performed, 2026-11-06 (Asia/Tokyo).** Full local package
compilation/linking succeeded, including navigation, all controller plugins,
robot model, perching core and available Gazebo plugin. Sixteen C++ regressions
passed: task twist/wrap/nonzero starts, both stroke directions, finite blocked
lead, measured +10/+9 distinction with stricter effective tracking, complete
compliance/release/execution flow, stale feedback/wrong session, expired online
activation, C2 continuation, removal of compliance from measured replanning seeds,
contact/baseline/limits, mass/energy/gravity/observer signs, extra-module counting,
moving-gimbal momentum, worker cancellation, and current URDF FK/QP/dynamics.
Five Python checks passed for exact UInt64 wire serialization, invalid session
types, and hardware offline/online plus simulation launch-parameter resolution.
Launch parsing only expanded configuration/Xacro; no nodes were started.

Release-mode host samples for a current-URDF +10-degree request with 30-degree yaw
took roughly 0.29–0.32 seconds to prepare, and a single articulated dynamics sample
roughly 1 ms. These are unit-check wall times on this computer, not onboard profiling
or a comparison with the historical bag runtimes. The parser reports the existing
dummy root inertia limitation (root mass 0.00001 kg); the original RobotModel also
excludes that root. Geometry was not changed to suppress the warning.

The reusable offline runner is `bash test/run_roll_yaw_checks.sh` from the existing
ROS/dependency environment. It creates its own `/tmp/gimbalrotor-checks.*` directory
and retains configuration/build logs, expanded URDF and test XML/logs. This session's
artifacts are in `/tmp/gimbalrotor-checks.8gBJ8o`. `git diff --check` was also used.
The ROS flight rostests, simulator, real middleware timing and robot were not run.

Further validation needs calibrated active-joint output torque/load signs, physical
task-torque sign checks in both directions, command-vs-realized FC wrench measurements,
tool/inertia/module verification, no-contact bias across posture, input/skew/dropout
bags, CPU/deadline profiles on the flight computer and measured compliance/settling
tests. Local mathematical and compilation checks do not establish force accuracy,
flight stability, collision safety or successful cutting under persistent resistance.

**File-by-file changes.**

| File | Change |
|---|---|
| `CMakeLists.txt` | Builds dynamics/admittance core sources, links diagnostic messages and installs the command helper |
| `package.xml` | Adds diagnostic_msgs build/runtime dependency |
| `config/beetle/RollYawPerching.yaml` | Adds online/deadline, measured-progress and scalar admittance settings; preserves existing hardware limits |
| `launch/bringup.launch` | Exposes and applies QP/online startup arguments in roll–yaw mode |
| `launch/roll_yaw_perching.launch` | Forwards both startup arguments |
| `include/gimbalrotor/control/gimbalrotor_controller.h` | Declares the input-estimate publisher |
| `src/control/gimbalrotor_controller.cpp` | Publishes the labelled FC-mode command/model wrench estimate; leaves control/observer semantics intact |
| `include/gimbalrotor/gimbalrotor_roll_yaw_perching_navigation.h` | Declares independent worker, source, compliance, goal-diagnostic and timing state |
| `src/gimbalrotor_roll_yaw_perching_navigation.cpp` | Integrates lifecycle, worker snapshots/results, measured effective tracking, topic enable, sole final reference, acknowledgements and diagnostics |
| `include/gimbalrotor/roll_yaw_cutting_trajectory.h` | Stores immutable request convention, measured progress, endpoint flags, continuation and execution settings |
| `src/roll_yaw_cutting_trajectory.cpp` | Adds cooperative preparation, C2 continuation, bounded measured phase, original-goal completion and detailed guard failures |
| `src/roll_yaw_perching_kinematics.cpp` | Caches independently copied articulated FK solver/buffers without changing FK/CoG equations |
| `include/gimbalrotor/roll_yaw_settling.h` | Bounds settling sample storage |
| `include/gimbalrotor/roll_yaw_task.h` | Implements continuous task-frame pitch and directed filtered progress |
| `include/gimbalrotor/roll_yaw_worker.h` | Implements bounded persistent workers, cancellation and shutdown lifecycle |
| `include/gimbalrotor/roll_yaw_dynamics.h` | Declares copied articulated dynamics and generalized momentum observer |
| `src/roll_yaw_dynamics.cpp` | Implements link energy, mass, generalized inputs and task torque residual |
| `include/gimbalrotor/roll_yaw_admittance.h` | Declares scalar dynamics/contact state and compliant synthesis interface |
| `src/roll_yaw_admittance.cpp` | Implements baseline, filtering/hysteresis, bounded release/integration, consistent IK derivatives and anti-windup |
| `scripts/keyboard_command/roll_yaw_command.py` | Sends exact current-session commands and checks acknowledgement |
| `test/CMakeLists.txt` | Registers the core gtest target; existing flight tests remain available but were not run |
| `test/roll_yaw_core_test.cpp` | Adds mathematical, execution, worker and actual-URDF regressions |
| `test/test_roll_yaw_interfaces.py` | Adds offline ROS wire and launch checks |
| `test/run_roll_yaw_checks.sh` | Reproduces isolated local build and offline verification |
| `test/ROLL_YAW_IMPLEMENTATION.md` | Records the audit, equations, operational behavior, schema, verification and remaining limitations |
