# Roll–yaw perching orientation trajectories

This plugin plans body orientation around a fixed external passive hinge. It does
not actuate a saw, regulate contact force, or certify flight/cutting dynamics.
Existing pitch-only and pitch-first navigator implementations are unchanged.

## Frames and mathematics

`W` is the estimator world frame; `B` is `base_link`; `C` is `roll_hand_link`;
`P` is the calibrated physical hinge frame. `G` is the controller's CoG frame.
`N` is the **measured body orientation at the current contact lock**, frozen for
that session. It is not automatically gravity-level or world-aligned.

Coordinates are `eta = [alpha, q_roll, q_yaw]`. Alpha is zero at each fresh lock,
with positive rotation about the configured unit axis **expressed in P**. Active
coordinates are URDF/model radians; `Servo.yaml` alone handles motor polarity,
scaling and offsets. Servo IDs remain 6 (yaw) and 7 (roll).

The KDL chain supplies `T_BC`; calibration supplies `T_CP`:

```
T_BP(q)    = T_BC(q) T_CP
T_WP_lock  = T_WB_measured T_BP(q_measured)
T_WB(eta)  = T_WP_lock Rot_P(axis, alpha) inverse(T_BP(q))
R_align    = R_WN Rz(body_yaw) Rx(body_roll)
R_des(t)   = R_align Ry(theta(t))
omega_des  = [0, theta_dot, 0]         (desired body coordinates)
```

Alignment is an SO(3) geodesic with a quintic time law to
`R_align Ry(pitch_start)`. Cutting uses cosine acceleration/deceleration ramps
and constant angular feed, with direction determined by `pitch_end-pitch_start`.
The angle profile is C2 and terminates at the specified endpoint. Multiplication
order is never replaced by independent world Euler interpolation. The body-local
Y saw-plane normal is constant during the ideal local pitch stroke.

If `J_BC,omega` is KDL's angular Jacobian expressed in B, the fixed-pivot body
Jacobian is

```
J = [ R_BP axis_P, -J_BC,omega[:,roll], -J_BC,omega[:,yaw] ]
```

Signed passive angle is recovered by quaternion swing/twist projection of
`R_WP_lock^T R_WB_measured R_BP(q_measured)` onto the calibrated hinge axis.
Residual rotation and pivot translation errors are checked separately. The
reported passive rate projects measured body angular velocity minus measured
active-joint angular contributions onto the hinge axis. It is not an Euler pitch
rate or an unfiltered derivative of alpha.

The differential IK uses `e = Log(R_FK^T R_des)` and
`omega_cmd = R_FK^T R_des omega_des + K e`. Its positive-definite objective is

```
0.5 (J u - omega_cmd)^T W (J u - omega_cmd)
+ 0.5 lambda ||u||^2 + 0.5 mu ||u-u_previous||^2
```

All 27 free/lower/upper active sets are enumerated deterministically; accepted
solutions satisfy KKT conditions. Each component's interval is the intersection
of velocity, next-position and inter-sample acceleration bounds. Singular values
are checked independently of regularization. There are no free-root QP variables.

Planning integrates QP solutions, reconstructs FK, and validates nonlinear tilt,
pivot, conditioning, orientation and angular-rate residuals. Failed attempts are
regenerated with doubled durations. Accepted coordinates use C2 quintic Bezier
interpolation. Bernstein control-point bounds certify position, velocity and
acceleration limits throughout each interval; nonlinear checks also sample four
subintervals. Half the velocity/acceleration budget is reserved for path-preserving
pause/cancel deceleration. A plan exceeding the configured phase duration or
retry count is rejected, without executing its partially generated path.

The inertia-weighted CoG is recomputed from the full copied KDL tree, including
extra modules. The repository defines `R_BG = inverse(getCogDesireOrientation())`;
it does **not** define G using principal inertia axes. This convention is captured
at lock. All FK pose derivatives include the configuration-dependent CoG offset.
The pure callback interface also supports a changing CoG orientation; tests cover
that case. The executor sends CoG angular rates in **desired G**, not raw body-Y
rates. Translational velocity/acceleration are in W.

## ROS configuration and operation

Select `airframe:=beetle roll_yaw_perching:=true` in `bringup.launch` only after
independent physical validation. This selects the roll–yaw mechanism, the ordinary
`gimbalrotor_controller`, and its existing **host allocation** path
(`gimbal_calc_in_fc: false`) so rate/acceleration feed-forward contributes to full
thrust/gimbal commands. The FC allocation path sends roll/pitch angle references
without these host rate references. No controller/firmware algorithm is added.

The default configuration intentionally cannot lock. Supply calibrated
`pivot_offset`, `pivot_rpy` (ZYX), and signed unit `hinge_axis` under
`navigation/roll_yaw_perching`, then set `calibration_confirmed: true` and restart
the navigator. A zero offset is acceptable only if explicitly calibrated as zero.
No geometry is borrowed from a different mechanism.

The body world-Z tilt limit is the **minimum** of `body_max_tilt` and
`navigation/max_target_tilt_angle`. Both default to 0.2 rad. Thus the example
20-degree stroke and 30/45-degree rolls are rejected by default. Both limits must
be explicitly changed following independent validation to use a larger envelope.
The existing `max_target_tilt_angle` key was not enforced by BaseNavigator; this
plugin reads and enforces it. Pitch-only-specific pitch limits do not apply here.

All commands below are descriptions; no ROS commands or nodes are run by the
unit tests. Topics are relative to the robot namespace:

1. In HOVER with fresh, settled feedback, enable via existing `perching/enable`
   (`std_msgs/Bool`). Capture body, active joints, physical pivot, N and a new
   monotonically increasing lock identity.
2. Read the latched `perching/roll_yaw/session` (`std_msgs/UInt64`). Every trajectory
   action requires that value in its `data` field, including cancellation.
3. Set `navigation/roll_yaw_perching/request/{body_yaw,body_roll,pitch_start,
   pitch_end,pitch_rate,pitch_acceleration}`. Units are rad, rad/s, rad/s²;
   rate and acceleration are positive magnitudes.
4. Send the session ID to `perching/roll_yaw/prepare`. `REZERO` denotes reference
   preparation, **not physical servo zeroing**. An immutable model snapshot is
   planned on a worker thread while navigation keeps holding contact. Both phases
   must pass before the plan becomes available in `CONTACT_LOCKED`.
5. Send the ID to `/align`. `PRE_ALIGN` runs the alignment and waits for measured
   settling before `READY`. Only then send the ID to `/start` for `EXECUTING`.
6. `/pause` decelerates along the accepted path into `PAUSED`; `/resume` accelerates
   along the same path. `/cancel` decelerates and discards the plan into
   `COMPLETED_HOLD`. Re-preparing from that hold is allowed.
7. Normal completion settles into `COMPLETED_HOLD`. Disable/enable creates a new
   physical lock. Legacy empty relock messages do not recapture an attached body.

All action topics above have the `perching/roll_yaw` prefix and type UInt64. The
prefix is configurable. Delayed plans/actions from older locks cannot execute.
Parameter requests are copied when prepare is received; later parameter changes
do not alter a running plan.

One navigation update computes one reference and writes named roll/yaw **position**
commands to `joints_ctrl` plus body/CoG controller targets. Alpha is never sent to
a servo. The synchronized baselink setter bypasses legacy orientation interpolation
and does not reset angular rates. This is software synchronization within one
update, not a hardware-atomic bus transaction.

`uav/nav`, `target_pose`, `target_path`, direct trajectory generation, external
baselink targets and control-mode changes cannot take ownership while attached.
Conflicting commands fault; joystick pose changes are suppressed while emergency
stop/land handling remains available. Existing navigators opt out of these guards
and retain their prior behavior. The shared base header adds virtual hooks, so
**rebuild dependent navigation/controller plugins before deployment**; do not mix
old binaries with the rebuilt base library.

## Feedback, diagnostics and faults

The adapter consumes stamped `joint_states` and the existing estimator
`uav/baselink/odom`. Body frame IDs must match configured `world_frame`/`body_frame`;
launch supplies the latter using `robot_ns`. Zero timestamps, malformed/duplicate
joint names, invalid quaternions, stale/out-of-order samples, excessive timestamp
skew and missing estimator position status prevent execution. Position-only joint
feedback needs two valid timed samples to derive a velocity. Messages containing
only unrelated gimbal joints are ignored.

Faults freeze the last accepted contact-consistent pose with zero derivatives;
they do not start a recovery movement or unperch. Emergency stopping may change
velocity abruptly; the smooth pause/cancel path is for valid feedback and operator
requests. HOVER/lock loss latches output inhibition; returning to HOVER does not
resume the old reference. Explicit disable and a fresh settled enable are needed.
The inherited emergency landing/halt behavior remains authoritative.

Diagnostics under the prefix:

- `state`: state and rejection/fault reason; `session`: lock identity.
- `body_reference`, `body_measured`: stamped W pose and body-frame angular rate.
- `cog_reference`: W pose and G-frame twist, matching Odometry conventions.
- `diagnostics`: labelled radians/metres/seconds snapshot, desired/measured
  coordinates, coordinate rates, passive rate estimate, feedback ages/validity,
  physical pivot and hinge residuals, planned orientation residual, QP plan status,
  scaling attempt, phase time, ideal body quaternion/rate, body/CoG accelerations
  and output-inhibition status.

## Verification and remaining limits

Pure GoogleTests cover the repository-expanded KDL chain, inertia-based CoG
reconstruction, fixed pivot, nonzero-state analytic/finite-difference Jacobians,
passive angle/residuals, random box-QP KKT checks, singular/infeasible/nonfinite
inputs, Cases A/B/C and reverse strokes, ideal blade normal and body/world rates,
changing CoG-frame derivatives, interpolation bounds, time scaling, unsafe default
rejection, lock/alignment/start/finish/pause/resume/cancel, stale sessions/plans,
stale feedback and state/pivot faults. All three mechanism coordinates are checked
to move during the requested sweeps.

`test/roll_yaw_integration_checks.py` parses launch/XML/YAML/Xacro without starting
a node/master. It checks mechanism selection, mutual exclusion, ordinary host
controller selection, safe defaults, plugin registration and unchanged servo IDs.
The actual-URDF C++ test accepts `ROLL_YAW_TEST_URDF`; it skips that one test if no
expanded model is supplied.

No simulator, ROS master, robot node, flight, contact experiment or saw operation
is part of these checks. Live topic timing, rotor authority, contact stiffness,
tracking performance and host-allocation operation remain to be validated. There
is no collision-distance query backend in this navigator; accepted trajectories
are **not collision-certified**. Nonlinear constraints are sampled, rather than
proven over the continuous path. Other articulation is snapshotted; live CoG drift
beyond `cog_model_tolerance` faults. These are kinematic plans, not dynamic or
contact-force feasibility guarantees.

The current URDF has the old pitch-only/pitch-first joint definitions inside XML
comments. Expanding those mechanisms produces disconnected links and no active
pitch joint. This pre-existing issue is not repaired here; their navigator source
files remain unchanged. Launch now explicitly disables the default roll–yaw
mechanism when selecting those variants, as required.

## Local verification record and file manifest

Verification used the existing ROS 1 installation at `/opt/ros/one`, roscpp
1.17.0, GCC 11.4.0 and CMake 3.22.1, with the existing workspace dependencies.
No packages were installed or system settings changed. Build/devel/log artifacts
are isolated in the repository's `.roll_yaw_verify/` directory (CATKIN_IGNORE).

Built the full `aerial_robot_control` package and the `gimbalrotor_navigation`,
`gimbalrotor_controller`, `gimbalrotor_robot_model`, `roll_yaw_perching_core` and
`roll_yaw_perching_test` targets. Ran 13 GoogleTests (including the expanded URDF,
no skipped tests) and 3 launch/configuration tests successfully. Also loaded all
rebuilt libraries using the normal estimator/control host dependencies and
robot-model-before-controller ordering. Built `aerial_robot_base_node` against
this overlay without running it. The host package's unrelated default ALL target
attempted desktop-icon installation outside the repository and was blocked by the
filesystem sandbox; building the explicit executable target succeeded without
those installation actions. Independent dlopen of those existing
libraries is not sufficient: the repository has shared host-symbol dependencies.

Re-run the non-actuating tests from the repository root:

```bash
source .roll_yaw_verify/devel/setup.bash
ROLL_YAW_TEST_URDF="$PWD/.roll_yaw_verify/use_roll_yaw_joint.urdf" \
  .roll_yaw_verify/devel/lib/gimbalrotor/roll_yaw_perching_test
ROS_HOME="$PWD/.roll_yaw_verify/ros" PYTHONDONTWRITEBYTECODE=1 \
  python3 robots/gimbalrotor/test/roll_yaw_integration_checks.py
```

Modified source/configuration files:

- `aerial_robot_control/include/aerial_robot_control/flight_navigation.h`
- `aerial_robot_control/src/flight_navigation.cpp`
- `robots/gimbalrotor/CMakeLists.txt`
- `robots/gimbalrotor/include/gimbalrotor/gimbalrotor_navigation.h`
- `robots/gimbalrotor/src/gimbalrotor_navigation.cpp`
- `robots/gimbalrotor/launch/bringup.launch`
- `robots/gimbalrotor/plugins/navigation_plugin.xml`

Created files (under `robots/gimbalrotor/`):

- `include/gimbalrotor/roll_yaw_perching_kinematics.h`
- `src/roll_yaw_perching_kinematics.cpp`
- `include/gimbalrotor/roll_yaw_orientation_qp.h`
- `src/roll_yaw_orientation_qp.cpp`
- `include/gimbalrotor/roll_yaw_cutting_trajectory.h`
- `src/roll_yaw_cutting_trajectory.cpp`
- `include/gimbalrotor/gimbalrotor_roll_yaw_perching_navigation.h`
- `src/gimbalrotor_roll_yaw_perching_navigation.cpp`
- `config/beetle/RollYawPerching.yaml`
- `test/roll_yaw_perching_test.cpp`
- `test/roll_yaw_integration_checks.py`
- `README_roll_yaw_perching.md`
