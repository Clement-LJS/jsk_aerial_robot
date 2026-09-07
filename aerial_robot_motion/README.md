# aerial_robot_motion

Online, constrained whole-body **reference generation** for ROS1 JSK aerial robots.
Cartesian commands update a nominal tool pose; Cartesian admittance optionally
modifies that pose; cost and constraint plugins assemble one velocity QP for each
new measured state. A validated solution becomes one body/joint reference step.
The existing navigation, flight and servo controllers execute those references.

## Validation of the supplied specification

The architecture is implementable on this branch, with these necessary details:

* Work remains on `develop/qp_solver`, as explicitly requested, overriding the
  document's older `qp_solver` branch name. No commit, push, fetch or branch change
  is part of this implementation.
* A transformable model's Cartesian Jacobian can include internal/gimbal joints
  and use a desired-CoG frame. The motion adapter instead uses the model's KDL
  tree and explicitly selects `getLinkJointIndices()` columns. Measured internal
  joint positions are retained for forward kinematics, but are not commanded.
* Root velocity variables are expressed in **world axes**. Orientation therefore
  uses left multiplication by the exponential, rather than the document's
  body-angular-velocity form with right multiplication.
* The angular projector `I-aaᵀ` has rank two. Two independent perpendicular
  basis rows provide exactly five contact equalities with the translation rows.
* Navigation positions are CoG positions; `FlightNav.target=BASELINK` alone does
  not convert them. The bridge uses the next joint configuration and root pose
  to calculate the next CoG pose. Controller capability and HOVER state are checked.
* The supplied rigid Beetle URDF has no articulated link joints. The intended
  **multilink Beetle** uses the same `6+n` implementation as other articulated
  robots; its actual URDF/model, tool/contact geometry and servo mapping must be
  supplied through its robot bringup. The rigid example is retained separately
  as an available-model check, not a substitute for that multilink configuration.

Reference studied: [motion_planning, revision 6772bb19](https://github.com/ut-dragon-lab/motion_planning/tree/6772bb191faadf7ea3217374f2ac328aba536182/aerial_motion/differential_kinematics),
especially its additive costs, constraints and qpOASES backend. Its iterative IK,
robot-specific casts and path playback were not ported. The
[root-perching manipulator paper](https://arxiv.org/html/2405.12125v1) provides
architectural context; its contact-force controller is outside this package.

## State, frames and QP

For `n` model-provided articulated link joints, the decision variable is

```
x = [v_root_world(3), omega_root_world(3), qdot_link(n)]
minimize 0.5 xᵀ H x + gᵀ x
subject to lb <= x <= ub, lbA <= A x <= ubA
```

For `n=0` the same implementation uses six variables. A Cartesian task adds
`H += 2 Jᵀ W J`, `g += -2 Jᵀ W v_desired`; state-velocity and nominal-posture
costs are additive. A positive configurable diagonal regularization keeps the
Hessian well conditioned. All direct bounds are intersected.

Tool and contact poses include both configurable translation and orientation
offsets. Their world Jacobians use the velocity of each frame's origin:

```
J = [ I, -skew(p_point_world - p_root_world), R_world_root J_KDL_linear
      0,  I,                                R_world_root J_KDL_angular ]
```

Only the selected link-joint columns enter the KDL blocks. The task orientation
error is `Log(R_target R_measuredᵀ)`, not Euler subtraction. Cartesian gains,
weights, masks and twist saturation can use world axes or measured tool axes
(`costs/cartesian_pose/axes_frame`). Six-axis arrays are `[x,y,z,rx,ry,rz]`.

One integration step is anchored at current feedback:

```
p_next = p_measured + dt v_world
R_next = Exp(dt omega_world) R_measured
q_next = q_measured + dt qdot
```

`dt` is the interval between processed odometry stamps. Repeated sensor samples
do not cause additional solves. An invalid or excessively long interval suppresses
that update. No offline path or internal target-reaching IK loop is generated.
qpOASES `SQProblem` hot-starts changing Hessians and constraint matrices, retaining
their backing storage; changes in dimension/contact row count reinitialize it.

## Hard constraints and failure behavior

* `StateLimit`: world root linear/angular component speeds and per-component
  translation/rotation step limits.
* `JointLimit`: URDF mechanical and velocity limits, configured velocity caps,
  and a damper near the inner safety margin. Retreat from a limit remains possible.
* `RevoluteContact`: capture the current contact position and hinge direction
  using `~perching/enable`. Three rows enforce
  `J_v x = Kp (p_locked - p_current)`. With orthonormal perpendicular rows `B`,
  two rows enforce `B J_omega x = Kr B e_axis`. `e_axis` is the shortest rotation
  aligning the current hinge direction with the locked direction; rotation about
  the hinge is free, even after substantial accumulated hinge rotation.

`hinge_axis_frame` can be `contact` or `world`. The hinge vector is normalized and
cannot be zero. Engage locks only after fresh, complete feedback; repeated enable
does not relock. Engagement resets the nominal tool target and admittance state.
Disable removes contact rows without changing the robot's legacy controller.

The velocity equality is a local kinematic approximation. Finite-step nonlinear
contact error is checked before publishing, with configurable position and angle
tolerances; stabilization corrects small accumulated drift on following updates.
It does not guarantee physical contact under disturbances or tracking error.

Invalid dimensions, NaN/Inf, missing joints, stale/inconsistent timestamps,
unsupported plugins, infeasibility and nonlinear contact-limit violations stop new
reference publication and generate diagnostics. Existing position references are
retained by the navigator; no previous velocity feedforward is left running by
this bridge. This node is separate from the flight-control process.

## Cartesian commands and admittance

Under the default node namespace `/ROBOT/motion`:

| Interface | Type / purpose |
|---|---|
| `command/pose` | `geometry_msgs/PoseStamped`: absolute nominal tool pose; empty frame means world |
| `command/increment` | `geometry_msgs/TwistStamped`: **increments**, meters/radians, not velocity; empty frame uses `command/frame` |
| `command/reset` | `std_srvs/Trigger`: reset target to current measured tool |
| `perching/enable` | `std_srvs/SetBool`: capture/release passive revolute contact |
| `admittance/enable` | `std_srvs/SetBool`: enable/disable compliance |
| `admittance/reset` | `std_srvs/Trigger`: disable and return offset to zero at bounded speed |
| `external_wrench` | default private `geometry_msgs/WrenchStamped` input; configurable/remappable |
| `reference/body` | `nav_msgs/Odometry`: next root pose and root-frame twist |
| `reference/joints` | `sensor_msgs/JointState`: matching timestamp and link-joint positions/velocities |
| `status` | `diagnostic_msgs/DiagnosticArray`: validity, solver status/time, dimensions, plugins, Cartesian residual and contact error |

Literal command/compliance frame `tool` means the current configured tool including
its fixed offset. Other frames use TF at the message stamp. Relative rotations
accumulate on the nominal target using measured command-frame axes.

Admittance precedes the QP and implements diagonal
`M xddot + D xdot + K x = S (wrench - wrench_reference)` with backward Euler:

```
v_next = (M v + dt (F - K x)) / (M + dt D + dt² K)
x_next = x + dt v_next
```

Displacement and velocity are bounded per axis. `axes` is a numeric 0/1 mask;
`[0,0,0,0,1,0]` enables tool pitch only. The compliant displacement modifies the
nominal pose; angular feedforward is intentionally zero because a finite rotation
vector's derivative is not exactly angular velocity. The QP still enforces contact.

Wrench conversion includes the moment arm:
`f_target = R f_source`, `tau_target = R tau_source + p × R f_source`.
The compliance frame's **origin as well as orientation** matters. Stale or
untransformable wrench contributes zero input, without subtracting a stale bias.
If the compliance frame itself is unavailable, correction is held until it can be
defined. Disable/reset returns the offset at bounded speed; re-enable during that
return preserves the remaining offset to prevent a jump. Initial enable starts at
zero. Nonpositive mass, negative damping/stiffness and unreasonable `dt` are rejected.

Keyboard example:

```bash
rosrun aerial_robot_motion keyboard_motion.py \
  _command_topic:=/gimbalrotor/motion/command/increment \
  _frame:=tool _rotation_step_deg:=0.5
```

`w/s`, `a/d`, `r/f` request ±X/Y/Z. `u/j`, `i/k`, `o/l` request ±roll/pitch/yaw.
Six `k` presses request −3° with the default 0.5° step. The script sends no body,
joint, gimbal or perching-geometry commands.

## Multilink Beetle and other robots

The core has no Gimbalrotor, DRAGON or Hydrus compile dependency, concrete-model
cast or robot-name branch. It loads `robot_model_plugin_name` in the robot namespace
(or `~model_plugin`) using the existing `aerial_robot_model` plugin mechanism.
The transformable base interface supplies link-joint names, KDL indices and limits.
Current model implementations select their own link joints; arbitrary future
joint naming must be exposed correctly by that model interface.

For a multilink Beetle, prepare its actual URDF and compatible robot-model plugin,
complete measured joint states, and the servo group's named link-joint command
mapping in the robot package. Then select the tool and physical hinge frames:

```bash
roslaunch aerial_robot_motion gimbalrotor_qp.launch \
  robot_ns:=gimbalrotor tool_frame:=YOUR_TOOL_LINK contact_frame:=YOUR_CONTACT_LINK
```

Parent-link plus calibrated `tool_offset_xyz/rpy` and `contact_offset_xyz/rpy` can
be supplied in a custom YAML via `config:=...`. The multilink example prefers
joint motion over body motion. Its six-axis task is soft: a target unreachable
under contact produces a residual. Deweight appropriate translational axes when
requesting a rigid hinged tool to rotate without prescribing its accompanying arc.

The existing rigid Beetle can be inspected separately using
`rigid_beetle_qp.launch`; `start_robot:=true` optionally starts its existing bringup
with the plain navigator and `use_saw:=true`. Its contact offset reuses the existing
navigation calibration. The `saw` frame is the URDF origin, not an assumed blade tip.
This is a six-variable example, distinct from the requested multilink robot.

Generic `aerial_robot_motion.launch` takes `robot_ns` and `config`. Examples include
`quadrotor.yaml` and `transformable.yaml`. The normal xyz/yaw navigator does not
accept independent roll/pitch targets: `bridge/full_attitude=false` adds two
baselink-angular-velocity equalities, including joint contributions. Enable full
attitude only for an existing compatible navigator/controller, such as Gimbalrotor's.
Controller capabilities and servo mapping still need verification on each robot;
structural model compatibility is not a claim of tested flight behavior.

## Execution and legacy coexistence

All launch files default to `publish_commands:=false`: the node publishes preview
references and diagnostics. To execute, set it true when launching in the robot's
namespace. Execution requires fresh HOVER flight state (`hover_state=5`) and
paired fresh baselink/CoG odometry. This is necessary because the existing navigator
otherwise rejects body commands while a servo could still accept joint commands.

Inputs normally resolve in the robot namespace:
`uav/baselink/odom`, `uav/cog/odom`, `joint_states`, `flight_state`.
Odometry `child_frame_id` must identify the configured baselink. The adapter converts
baselink pose to the KDL root and recovers the controller's desired CoG orientation
from the paired odometry. Command output uses `uav/nav` (`FlightNav`, CoG position
mode) and configured `joints_ctrl` (`JointState`). For `n=0` no servo command is sent.

Body and joint references share one solution timestamp. ROS topic transport does
not guarantee atomic consumption by separate controllers. Root twist remains in
the preview; execution uses position mode to avoid incorrect root-to-CoG velocity
feedforward. No PWM, Spinal, CAN or motor command is produced.

Use the plain Gimbalrotor navigation plugin for QP execution. The old perching
navigator rewrites targets when enabled. The example also listens to the existing
latched `perching/enable` topic as a command inhibit; it does not enable that old
mode. The new contact service is `/ROBOT/motion/perching/enable`.
The existing bringup gained only two optional arguments (`flight_navigation_plugin_name`
and `model_options`); its previous defaults and perching/controller code are preserved.

`fixed_joint_positions` can describe genuinely fixed, uncommanded model joints
that do not appear in feedback (e.g. the rigid Beetle's configuration-only dummy
servos). It cannot replace measured link-joint positions and should never be used
to conceal a missing physical articulation or moving gimbal measurement.

## Build and tests

C++17, ROS1/catkin, Eigen, KDL and the existing aerial-robot model/messages are
required. qpOASES headers are confined to the solver source. CMake first accepts
`QPOASES_INCLUDE_DIR` and `QPOASES_LIBRARY` or searches installed prefixes. Otherwise
it builds qpOASES 3.2.0 from an immutable HTTPS archive with a verified SHA256,
entirely in the build directory. No git operation or system installation is used.
The external dependency retains its upstream LGPL license.

```bash
catkin build aerial_robot_motion
catkin run_tests aerial_robot_motion
catkin_test_results
```

For an isolated existing-workspace check, configure this package with `cmake -S`
and a build/devel/install prefix under `/tmp`, sourcing the existing devel space.
Provide the already-built qpOASES include/library paths to avoid another download.
ROS tests require a usable localhost XML-RPC/TCPROS setup; override both `ROS_IP`
and `ROS_HOSTNAME` when the shell contains an obsolete network address.

Tests cover additive QP assembly, intersected/stacked bounds, changing-matrix
hot-starts, infeasibility, Cartesian direction/residual, joint-limit retreat,
five-direction contact, wrench transforms, stable bounded admittance, command
accumulation, zero-joint models, and selected-column KDL finite differences.
An articulated two-link synthetic plant performs 400 measured-feedback updates
while tracking −3° tool rotation; both body and joint move and contact error is
reported numerically. The ROS integration test checks actual plugin loading,
navigation output, command/services, HOVER gating and stale-state rejection.

Local verification on `develop/qp_solver`: all **21 distinct test cases passed**
(the catkin XML summary also counts nested suite/wrapper records and reports 43,
with zero errors, failures or skips). The articulated perching test measured
maximum contact position error `5.02422e-7 m`, hinge-axis error `1.13495e-15`, and
final tool orientation error `4.61343e-9 rad`. These are synthetic perfect-tracking
results. The package built both with supplied qpOASES paths and with a fresh
download through the pinned ExternalProject fallback. Isolated installation and
both robot launch expansions also passed. Build/download/test artifacts are under
`/tmp/aerial_robot_qp_validation`, not in the source tree.

## Extensions and current limits

Add costs/constraints by deriving the corresponding `Base`, exporting the class
with pluginlib and registering XML. `initialize(nh, info)` validates configuration;
`update(context, problem)` contributes a cost, intersects bounds or appends rows.
Costs may expose a diagnostic residual. Configure unique plugin names with their
own `costs/NAME` or `constraints/NAME` parameter namespace. Unknown/unsupported
explicitly requested plugins fail startup clearly.

Optional static-thrust, joint-torque, CoG-momentum and stability plugins are **not
ported in this version**. Existing transformable APIs differ in physical-Jacobian
dimensions, virtual CoG conventions and internal-gimbal treatment across models.
They need verified capability adapters and velocity-level `dt` scaling before they
can be enabled safely; there are no placeholder constraints pretending to enforce
them. The required kinematic constraints operate independently of those APIs.

The actual multilink Beetle URDF/servo configuration is not present on this branch,
so deployment on that hardware remains unverified. There is no hardware/flight,
contact-force, friction, ZMP, collision, MPC, global-path or stability guarantee.
The current perch is only a kinematic passive revolute constraint. Published
timings are diagnostics; no hard real-time execution guarantee is made.

## File responsibilities

| Files | Function |
|---|---|
| `CMakeLists.txt`, `package.xml`, `cmake/qpOASES.cmake` | Build, package export, verified solver dependency, tests and installation |
| `include/.../core/motion_state.h` | Model metadata, measured state/context, whole-body result and SO(3) math |
| `include/.../core/qp_problem.h` | QP assembly and numerical/dimension validation |
| `include/.../core/solver_base.h` | Backend-independent solver interface |
| `include/.../core/plugin_utils.h`, `ros_conversions.h` | Validated parameter and pose/frame conversions |
| `include/.../core/model_adapter.h`, `src/core/model_adapter.cpp` | Plugin model, measured joint mapping, KDL FK/Jacobians and integration |
| `include/.../core/motion_core.h`, `src/core/motion_core.cpp`, `src/core/node.cpp` | Online loop, state gates, modes, plugin loading and diagnostics |
| `include/.../cost/base_plugin.h`, `include/.../constraint/base_plugin.h` | Plugin interfaces |
| `include/.../cost/{cartesian_pose,state_velocity,nominal_posture}.h`, corresponding `src/cost/*.cpp` | Cartesian tracking, motion preference and posture costs |
| `include/.../constraint/{state_limit,joint_limit,revolute_contact}.h`, corresponding `src/constraint/*.cpp` | Required hard constraints |
| `include/.../solver/qpoases_solver.h`, `src/solver/qpoases_solver.cpp` | Isolated qpOASES backend, persistent hot-start storage, result validation |
| `include/.../command/motion_command_manager.h`, `src/command/motion_command_manager.cpp` | Nominal Cartesian target and ROS command handling |
| `include/.../compliance/cartesian_admittance.h`, `src/compliance/cartesian_admittance.cpp` | Wrench handling and bounded Cartesian compliance |
| `include/.../execution/whole_body_reference_bridge.h`, corresponding `.cpp` | CoG navigation and synchronized named link-joint references |
| `plugins/cost_plugins.xml`, `plugins/constraint_plugins.xml` | Runtime plugin registrations |
| `config/default.yaml` | Generic defaults, limits, plugin selection and compliance settings |
| `config/examples/{quadrotor,transformable,gimbalrotor_perching,rigid_beetle_perching}.yaml` | Robot/mode configuration examples |
| `launch/{aerial_robot_motion,gimbalrotor_qp,rigid_beetle_qp}.launch` | Generic, multilink and existing rigid-model entry points |
| `scripts/keyboard_motion.py` | Cartesian keyboard increments |
| `test/main.cpp`, `test/test_{qp,command,admittance}.cpp` | Unit-test entry point and numerical tests |
| `test/model_kinematics.cpp`, `test/model.test` | Model/Jacobian and articulated online-contact validation |
| `test/runtime_smoke.py`, `test/runtime.test` | ROS integration test |
| `README.md` | Architecture, equations, integration, configuration and limits |
| `../robots/gimbalrotor/launch/bringup.launch` | Only modified existing file: optional navigator/model arguments |
