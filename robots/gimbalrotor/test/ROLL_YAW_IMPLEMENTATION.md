# Manual navigation and feedback recovery correction (2026-11-06)

The local source and the locally available `be4b999` object confirmed the
regression: manual admission ran full sampled FK validation while holding the
same mutex as feedback callbacks and navigation. Manual commands now construct
two-knot quintics with bounded analytic coefficient checks and endpoint FK only.
Sampling reuses the generated reference's desired attitude/rates, without a
per-key articulated-model closure. Full automatic geometry/IK/CoG/feed and
cut-start connection validation remains in the offline worker. All command
routes retain independent destinations and one canonical applied reference.

Joint/body candidates are validated before replacing the complete accepted
sample. Rejections cannot refresh or destroy that sample. Joint finite-difference
bootstrap uses separate advancing samples, including after a real outage.
Temporary observation unavailability holds the coherent pose, zeros command
feedforwards and resets motion/quiet clocks. Manual motion recovers after contact
and tracking checks; automatic motion retains its goal in PAUSED and requires
explicit resume. Stop/cancel/disable override waiting movement. Rezero retains
its measured/frozen-joint checks and timeout and cannot count an outage as
settling. True geometry/tracking/clock faults remain latched. Emergency holds
are not evidence of acceleration-continuous physical stopping.

`diagnostic_dynamics` defaults false and gates the diagnostic worker early;
external-wrench projection, explicit no-cut tare and path admittance remain
independent. Callback timing now includes mutex wait, validation and diagnostics.
The 0.15 s feedback timeout, 0.05 s synchronization tolerance, execution-step
bound, gains, actuator limits, firmware and other perching modes are unchanged.

Verification for this correction:

- Full Release ROS/catkin package configure, compile and link passed using the
  existing environment and `test/run_roll_yaw_checks.sh` build commands, with
  build/devel/install paths in a temporary directory. Built controller, robot
  model, roll-yaw core, navigation and Gazebo plugin libraries. No installation,
  workspace devel replacement, dependency changes or environment setup occurred.
- All **19 core tests** passed using the expanded current beetle URDF. Added
  actual polynomial retarget/derivative/stop checks, bag-7 yaw increment,
  moving automatic feedback pause/resume without phase catch-up, cancel,
  alignment pause, timestamp skew and true contact/limit/clock fault checks.
  Offline alignment, local-pitch preparation, full connection validation,
  measured completion and admittance tests passed. Manual construction remains
  available with a cancelled planning hook; the fully validated connection
  rejects it, verifying the helper separation.
- All **6 existing interface tests** passed, including real keyboard logic with
  mocked ROS publication and hardware/simulation launch resolution.
- All **6 temporary linked-navigator probes** passed with a private loopback ROS
  master and isolated test publishers. These directly called the compiled
  navigator callbacks/reference application methods with synthetic measurements
  and the real expanded URDF. They covered malformed, nonfinite, duplicate,
  out-of-order and wrong-frame rejection; empty-velocity bootstrap after a gap;
  the bag-7 command advancing; held controller targets and manual recovery with
  unchanged session/no catch-up; stop/disable; long loss without motion; real
  pivot/clock faults; skew and pending preparation cancellation; legacy vector
  and passive-pitch inputs; rezero timeout; and disabled diagnostic scheduling.
  Separate threads exercised 50 Hz joint/body feedback and repeated axis commands
  under the navigation mutex. A delayed offline worker did not block commands.
- Final run: actual manual callback maximum **0.290 ms**, including admission,
  mutex wait and diagnostics; concurrent-command maximum **0.184 ms** across
  41 feedback samples. Reference application plus diagnostics maximum **0.0184
  ms** in that probe. Real-URDF helper maxima were **0.0202 ms** construction and
  **0.0290 ms** sampling/checking. These are host measurements, not target-CPU
  real-time guarantees. The inherited legacy callback's outer mutex wait is
  outside the derived pitch handler's timer; keyboard axis/vector callbacks and
  `uav/nav` start timing before their mutex acquisition.
- Source call-site audit confirms full sampled validation is reachable only
  through offline `prepare`/`validatedMechanismPath`, never ordinary manual
  admission or control updates. Timeout/synchronization/step limits, publishers,
  keyboard/topics, firmware, allocator, gains and other perching architectures
  were preserved. No QP or calibration acknowledgement interface was added.
  `git diff --check` passed. Temporary probes, binaries, URDF, logs and build
  outputs were removed after verification; the private test master was stopped.

The supplied bag diagnosis was checked against local source and the historical
Git object; bags were not replayed or independently reanalysed. The probes do not
run the full flight-controller/main-loop process, sensor transport/spinner,
physical articulated-CoG drift, Gazebo physics, hardware, flight or cutting.
Controller input setters were exercised and coherent held position/velocity/
acceleration checked; downstream actuator behavior and real hardware timing
remain unverified. No commit, push, branch change or remote access was performed.

Files changed for this correction:

| File | Change and purpose |
| --- | --- |
| `src/gimbalrotor_roll_yaw_perching_navigation.cpp` | Candidate feedback caches, bounded manual sampling, observation hold/recovery, offline connection call, diagnostic opt-in and complete callback timing remove starvation and permanent timing faults. |
| `include/gimbalrotor/gimbalrotor_roll_yaw_perching_navigation.h` | Minimal wait/bootstrap/diagnostic state supports those local behaviors. |
| `src/perching_trajectory.cpp` | Separate analytic manual construction from full offline validation, share generated FK derivatives, validate candidates and pause/rebase the automatic executor on observation loss. |
| `include/gimbalrotor/perching_trajectory.h` | Declare split helpers and typed feedback status shared by navigator/executor. |
| `config/beetle/RollYawPerching.yaml` | Default-disable diagnostic dynamics without changing feedback or motion limits. |
| `test/roll_yaw_core_test.cpp` | Extend existing regressions to cover manual retargeting, automatic recovery and real faults. |
| `scripts/README.md` | Describe actual manual checks, waiting/recovery, explicit automatic resume and diagnostic default. |
| `test/ROLL_YAW_IMPLEMENTATION.md` | Record this correction's verification and remaining runtime limitations. |

The following older audit describes its own historical run; its counts and build
restrictions are not the results of this correction.

---

# Coordinated local-pitch cutting: local implementation audit

Current workflow correction: predictive thrust assessment/readiness and its opt-in
were removed. Accepted geometry plans automatically connect and wait for measured
READY. Optional external-wrench admittance uses prepare → READY → no-cut tare →
baseline ready → enable → start, with actual source validation and no calibration
acknowledgement. The checks/files below record the earlier implementation audit,
not a validation run of this correction.

Date: 2026-11-06. Starting branch: `develop/pochitab_passiveperching/main`.
Starting HEAD: `729c8fc1386ec1d116c189cce49bafbaef829bff`, exactly the supplied
reference; the checkout was clean. No branch change, commit, push, remote access,
ROS environment setup, catkin build, deployment, or running robot was used.

## Checked conventions and corrections

The body-to-contact chain is fixed mounting → yaw → roll → hand → configured
pivot. FK reverses it from the fixed perch, inserting the passive hinge before
active roll/yaw. `Kinematics::evaluate` computes
`T_WB = T_WP_lock * Rot(axis, alpha) * inverse(T_BP(roll,yaw))`, then
`T_WG = T_WB * T_BG(alpha,roll,yaw)` using the copied articulated inertia model.
The pivot position is invariant even when both active joints move. The analytic
body angular Jacobian has negative active-joint axes because this chain is
inverted. Alpha is lock-relative passive rotation, not body Euler pitch.
The nominal mounting yaw is 3.142, not exactly pi; the existing model retains it.
Near neutral, positive alpha decreases body pitch; positive local-body-Y feed
moves the forward saw-origin proxy downward. This sign is calculated along each
requested path rather than assumed globally.

The controlled beetle body is `fc`, whose origin is 0.06927 m above `base_link`
with the same orientation. The saw origin in `base_link` is
[0.05918, 0.00025, 0.01700] m. Its `fc` coordinate is therefore
[0.05918, 0.00025, -0.05227] m in this URDF. The implementation resolves the fixed
transform through the actual model, not hard-coded offsets. This is not a blade
center or tooth contact calibration. Body roll is not a certified blade-plane
inclination. No straight vertical cut or world-fixed blade plane is required.

`getCogDesireOrientation()` is the model's controlled CoG convention. The copied
articulated mass distribution sets CoG position; its orientation is not replaced
by principal inertia axes. Servo.yaml already supplies yaw `angle_sgn=-1`, roll
angle scale 0.001534, and the shared offset. The navigator uses URDF joint radians
and does not apply those hardware conversions a second time. URDF roll/yaw travel
is ±pi/2 and velocity 0.1 rad/s; configured bounds are intersected with them.
Configured acceleration caps and the passive range are assumptions to validate
on hardware, not evidence of motor capabilities.

## Implemented flow

1. Enable/relock captures the measured pivot/session through the existing base
   lifecycle. HOVER, coherent timestamped pose/joints, bounded mechanism state,
   pivot/hinge residuals, and articulated-CoG consistency protect the outputs.
2. Keyboard and mechanism-topic targets retain independent nominal destinations.
   A non-IK C2 mechanism transition uses analytic whole-interval joint bounds,
   endpoint FK/attitude rejection, and per-update body/CoG and rate checks. Its applied configuration produces all body/CoG derivatives and the
   active-joint commands. Rezero, final relock, hold, and key signs are retained.
3. Separate numeric body topics request `N Rz(yaw) Rx(roll) Ry(beta)`, with N the
   current lock's measured body orientation. Values are absolute intrinsic Z-X-Y
   coordinates relative to N. Untouched accepted body coordinates persist;
   beta and untouched values initially come from the measured posture. An
   offline damped-Jacobian IK path commands coordinated body/servo alignment.
4. Cut preparation captures fresh measured R0 and active joints. It reconciles
   small orientation/joint inconsistency only within configured tolerances and
   rejects larger inconsistency. A noisy aerial robot need not have zero rates:
   existing body/joint rate thresholds and endpoint settling settings are used.
   Preparation never changes to default yaw/roll. The cut is `R0 Ry(d)`.
5. Successive IK samples use the previous solution, nearest-seed angle wrapping,
   convergence and singularity checks, joint bounds and branch-step limits.
   Quintic interpolation uses Jacobian-derived velocity/acceleration. Bernstein
   control-point bounds cover the entire joint polynomial intervals. Body tilt,
   CoG/FC attitude, CoG derivatives, orientation/rate error and saw-origin world-Z
   direction are sampled at dt/4 or finer. These nonlinear sampled checks are
   not a mathematical continuum certificate. Half of velocity/acceleration
   capacity is reserved for execution time scaling. Failed requests report
   sample/interval and constraint details; scaling slows the request without
   changing its geometric goal.
6. The existing executor owns the offline path; no new state machine was added.
   All reference derivatives use one progress variable. Nominal lead is bounded
   by measured progress; sustained lag pauses advancement and eventually faults.
   Completion requires measured local pitch, full orientation, mechanism tracking
   and settling, not elapsed time. Start/end rates are zero. Prepared movement,
   changed session and accepted manual ownership invalidate incompatible plans.
7. One navigator publishes only active roll/yaw positions to `joints_ctrl`.
   `commandDesiredBaselinkPose` sends articulated CoG position, velocity,
   acceleration, attitude and angular derivatives and synchronizes body-to-CoG
   orientation for the existing fully actuated controller. It continues this
   coherent output for every valid roll-yaw lock, even if the generic base
   optional-hold switch is off. Stop/fault holds the applied contact-consistent
   reference; it never requests zero rotor thrust or zero joint angles.

## Requirement status and physical limits

| Requirement | Result |
| --- | --- |
| Local branch only, no commits/remotes/build setup | Satisfied; direct standalone checks only |
| Remove roll-yaw QP and online replanning | Removed solver files, source entry, configuration, launch arguments and continuation paths; no selectable fallback |
| Preserve keyboard and numeric mechanism targets | Implemented; independent axes, bounded accumulator/reversal, session/freshness checks, rezero/relock/hold retained |
| Numeric coordinated body yaw/roll alignment | Implemented on separate Float64 topics; intrinsic frame semantics and asynchronous acknowledgement documented |
| Fresh current alignment as cut start | Implemented; measured R0, active feedback, lock/session identity and movement invalidation |
| Local-body pitch, coordinated active joints, fixed pivot | Implemented and checked on synthetic and actual URDF chains |
| Downward feed sign | Checked for the rigid saw-origin proxy; physical tooth contact remains unverified |
| Smooth profile and joint/controller bounds | Implemented; whole-interval joint bounds and sampled nonlinear limits; physical tracking unverified |
| Thrust/vectoring feasibility | Not evaluated; no predictive planner gate |
| Optional cutting admittance | Initially disabled; READY cut, valid fresh external wrench and explicit no-cut baseline required to enable |
| Runtime feedback and completion | Existing gates plus manual/trajectory tracking and measured endpoint checks; no measured rotor saturation telemetry available here |
| Collision, structure, current/torque, dynamics, cutting loads | Not checked; no simulation/flight/hardware validation |

Firmware limits and controller allocation remain unchanged. Kinematic preparation
does not certify thrust, load, stability or actuator capacity. See the existing
README for the downstream clipping limitation.

The configured external-wrench input uses signed coordinated path work with
moment shifting. The command-derived wrench and articulated momentum observer
remain diagnostic only. Tare removes a no-cut baseline; it does not establish
physical cutting-force accuracy. Enabled source loss causes bounded stopping;
all body/CoG/joint references retain the same progress and original goal.

## Interfaces and usage

See [scripts/README.md](../scripts/README.md) for exact topics and runnable examples.
Keyboard: `e` enable, `j/l` yaw −/+, `a/f` roll −/+, `i/k` passive alpha +/−,
`z` rezero, `b` final relock, Space hold, `d` disable. Mechanism topics are
`alpha_target`, `roll_target`, `yaw_target`; body topics are `body_yaw_target`,
`body_roll_target`, all Float64 radians under `/gimbalrotor/perching/roll_yaw`.
Cut parameters are `request/pitch_increment`, `request/pitch_rate`, and
`request/pitch_acceleration` under `/gimbalrotor/navigation/roll_yaw_perching`.

The UInt64 session-tagged actions are `prepare`, `start`, `pause`, `resume`,
`stop`, `cancel`. `roll_yaw_command.py` obtains the exact current session and
checks admission on `command_ack`. Wait for asynchronous `state`/diagnostics:
`READY` means geometric preparation or completed measured body alignment, not
thrust feasibility. A busy worker rejects new requests without queueing them.
No explicit `align` action remains in the public cut workflow.

## Checks performed

- Built only the standalone existing core test executable directly with g++,
  Eigen, KDL, installed ROS libraries and gtest. No catkin/package build.
- Expanded the current beetle Xacro locally and supplied it to the test binary.
  All **17 core tests** passed, including yaw ±0.4 rad at body roll 0/pi/6,
  orientation IK/FK and pivot consistency, downward proxy sign and upward
  rejection, mechanism alignment followed by a cut, interpolation derivatives,
  manual limit reversal, the former thrust-start gate (since removed), moved prepared
  start, stale/session checks, blockage and measured completion. Legacy observer
  and scalar-admittance utility checks pass without implying runtime availability.
- All **6 interface tests** passed: UInt64 preservation/rejection, hardware and
  simulation launch resolution, actual keyboard independent-axis/reversal logic,
  stale feedback/new-session rejection. ROS publication was mocked for keyboard
  checks; no node, ROS master or actuator was started.
- Navigator C++ syntax-only check against already installed workspace headers.
  This does not establish a successful full package link or ABI compatibility.
- Source/call-site and build-reference audit, Python/launch parsing and
  `git diff --check`. Temporary executable/URDF artifacts were removed.

No physical pivot/saw calibration, full ROS integration, simulator run, flight
or real cutting was performed. Sampling resolution and configured controller
limits are software checks, not physical validation.

## Files changed and retained scope

Modified:

- `CMakeLists.txt`
- `config/beetle/RollYawPerching.yaml`
- `config/beetle/RollYawPerching_sim.yaml`
- `include/gimbalrotor/gimbalrotor_roll_yaw_perching_navigation.h`
- `include/gimbalrotor/perching_trajectory.h`
- `launch/bringup.launch`
- `launch/roll_yaw_perching.launch`
- `scripts/README.md`
- `scripts/keyboard_command/keyboard_roll_yaw_perching.py`
- `scripts/keyboard_command/roll_yaw_command.py`
- `src/gimbalrotor_roll_yaw_perching_navigation.cpp`
- `src/control/perching_admittance.cpp`
- `src/perching_trajectory.cpp`
- `test/ROLL_YAW_IMPLEMENTATION.md`
- `test/roll_yaw_core_test.cpp`
- `test/test_roll_yaw_interfaces.py`

Removed only after inspecting call sites:

- `include/gimbalrotor/roll_yaw_orientation_qp.h`
- `src/roll_yaw_orientation_qp.cpp`

`Limits` moved into the retained trajectory header; the singularity threshold
is now an independent planning field. FK/CoG, task progress, settling, worker,
base session/hold, rezero, actuator output and non-QP dynamics/admittance utilities
remain. Existing test files were updated; no new test files remain. The shared
controller, allocation, motor/servo configuration, URDF, Gazebo contact plugin,
package dependencies, other perching modes, bags and user data were preserved.
The shared launch change removes only the roll-yaw-specific obsolete arguments.
No unrelated QP functionality was removed; no solver dependency was dropped.
