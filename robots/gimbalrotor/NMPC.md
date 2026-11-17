# Experimental four-tilt servo NMPC

This is source preparation for Ubuntu 22.04, ROS One, STM32H7_v2 timer PWM
(`DSHOT=0`, `DYNAMIXEL=1`, `KONDO=0`) and four XC330-T181-T servos. It is
disabled at build, launch, profile and firmware levels. It has NOT been built,
tested, simulated, flashed or validated on hardware. Do not treat the supplied
quad URDF, MotorInfo, Servo profile or tuning weights as your calibration.

The implementation baseline is local `master_nmpc` / `master` at
`9b8bc02250e755e1493fae4afef4ae6c31b12d7d`. No branch, commit or remote was
changed. The editing host reports Ubuntu 26.04 and x86_64, with Python 3.14;
it is not the target Ubuntu 22.04/Python 3.10 environment.

## Prepared dependencies and compatibility

The user's follow-up requested dependency preparation as well as repository
edits. Sources were downloaded outside the robot checkout, without Git clone,
fetch, submodule operations, package installation or compilation:

* `/home/clement_wsl/acados-v0.5.0`: acados v0.5.0,
  commit `fd231104913cc9c305e0dff736281b91dc1dbc5b`.
* `external/blasfeo`: `9c2a8345f9de37fbb4b3cd84810b0250810de622`.
* `external/hpipm`: `945645e8c0ac473f3983fb3c7f22dbae50d864cc`.
* `interfaces/acados_template`: Python template from that same acados source.
* `interfaces/acados_template/tera_renderer`: v0.0.34,
  `c41c5a03a03a42d3ac5084ddcd415502f7721b05`, matching acados's `TERA_VERSION`.
* `NMPC_SOURCE_LOCK.json`: source URLs, commit pins and downloaded archive hashes.

The source-only downloader is `scripts/nmpc/prepare_sources.py`. It refuses to
vendor dependencies into this repository or overwrite different existing files.
The only skipped archive entry was HPIPM's optional experimental bibliography
symlink to `/home/andrea/syscop_bibtex/syscop.bib`. Optional QP backends and
their submodules were not downloaded; this controller uses HPIPM only.
`build`, `include`, `lib` and `bin` directories exist but contain no newly built
libraries or renderer. Headers currently reside in the source directories;
native installation will populate the include/lib prefixes.

Version choices follow the read-only
[beetle_omni README](https://github.com/Li-Jinjie/jsk_aerial_robot_dev/blob/develop/MPC_tilt_mt/robots/beetle_omni/README.md)
and [generation requirements](https://github.com/Li-Jinjie/jsk_aerial_robot_dev/blob/develop/MPC_tilt_mt/aerial_robot_control/scripts/requirements.txt):
acados v0.5.0, CasADi 3.7.0 and NumPy 1.26.4. PyYAML is needed to read the
profile. `acados_template` brings its own Python dependencies. Plotting,
pandas, ROS Python packages and the reference's additional robot controllers
are unnecessary for this generator. Python packages were not installed into
this host's Python 3.14; use an isolated native Python 3.10 environment later.

## Model, frames and interfaces

The existing `robots/quad/gimbalrotor.urdf.xacro` includes four independent
`gimbal1..4` revolute joints about local +X and `rotor1..4` thrust joints.
No Beetle, perching, gripper, saw, articulated joint or alternative-airframe
asset is imported. Its mechanical suitability still requires the owner's
confirmation: matching topology does not establish matching geometry/inertia.

State order is `[p_W(3), v_W(3), q_WG(w,x,y,z), omega_G(3), alpha_actual(4)]`.
Inputs are `[f1..4 in N, alpha_command1..4 in rad]`. Rotor order 1..4 maps
to the four `FourAxisCommand.base_thrust` elements and TIM1 CCR1..CCR4.
Physical wiring must be verified by the owner.

Runtime parameters have length 100: mass and full column-major inertia at
0..9; four 17-element rotor blocks at 10..77; reference state at 78..94;
reference thrust at 95..98; estimator gravity at 99. A rotor block contains
pivot position in CoG axes (3), zero-angle rotation (9, column-major),
rotor offset from the pivot in joint axes (3), signed drag ratio (1), and
measured servo time constant (1).

Geometry, current CoG, mass and full inertia come from the loaded robot model.
The prediction rotates both the thrust direction and the offset from each
gimbal pivot, rather than assuming the thrust point lies on the pivot.
CoG/inertia are frozen at their current values over one prediction horizon.
Tilting-link inertial coupling, joint-momentum terms and CoG migration over
the horizon are not included in this rigid-body MVP; their acceptability is
a hardware modeling assumption, not a validated fact.

Master's `rotor_direction * m_f_rate` is the **signed** drag ratio. Do not
negate it again to imitate the paper's `-d*kq/kt` notation. The supplied
master MotorInfo has `m_f_rate=0` while quad's URDF has `-0.0172`; the new
controller deliberately rejects this mismatch. Neither file was recalibrated.
The physical spin signs and drag ratio remain user-owned.

The [paper's public HTML, equations (2)–(10)](https://arxiv.org/html/2405.09871v2)
was inspected; no attached paper PDF was available. The simplest no-disturbance
servo model from Li's reference was inspected, not copied wholesale.
Dynamics use `qdot = 0.5*q_WG*[0,omega_G]`. Attitude residual is the vector
part of `conjugate(q_ref)*q`, expressed in target-body axes, as in Li's
implementation; this differs from the paper's world-frame multiplication
order. Its squared cost is invariant to quaternion sign. Measured quaternion
norm is checked and its hemisphere kept continuous. Rotation matrices are
normalized in prediction; predicted quaternion norms are checked after solve.

The stage cost has 16 state residuals (orientation contributes three) and
8 input residuals. Servo command cost penalizes `alpha_command-alpha_actual`.
The OCP uses NONLINEAR_LS, ERK with four stages, SQP_RTI, GAUSS_NEWTON and
PARTIAL_CONDENSING_HPIPM. The control period and prediction interval are both
0.025 s: 20 intervals give a provisional **0.5 s** horizon. This deliberately
avoids mixing the document's optional 0.1 s prediction interval with master's
40 Hz loop. No 100 Hz or VIM4 deadline claim is made.

Existing navigator position/velocity/acceleration and target-body angular
rate/acceleration are extrapolated across the horizon. A rank-checked
pseudoinverse provides references only. Reference thrusts are clipped to
physical bounds; angle references are unwrapped, bounded and rate limited
using the user-verified URDF velocity limits. Infeasible wrench references
produce warnings; only the solver's bounded first input is sent to actuators.

`uav/cog/odom` publishes world position and **world linear velocity**, but CoG
body angular velocity, despite the usual Odometry child-frame convention.
The controller follows the actual master publisher rather than rotating
linear velocity a second time. Raw Spinal quaternion order is xyzw; solver
order is wxyz. FC, base_link and CoG are not assumed colocated.

Freshness is checked against both ROS message stamps and wall receipt time.
Raw IMU and raw mocap are also checked because a fresh odometry publication
alone does not prove that its position or orientation source remains valid.
Master never sets the experiment-mode CoG rotation-status flag; the gate
instead requires calibrated ACTIVE IMU status and validated raw measurements.
This MVP intentionally accepts only the current IMU+mocap experiment mode.

Servo positions come from `servo/states` using exactly the servo bridge's
`angle_scale * angle_sgn * (raw_angle-zero_point_offset)` conversion and logical
indices. `gimbals_ctrl` still goes through the unchanged host servo bridge to
`servo/target_states`, `ServoControlCmd` and the existing Dynamixel transport.
ROS indices 0..3 are not necessarily physical Dynamixel bus IDs; the owner's
board inventory must establish that mapping. Per-servo/common feedback
filters and separate group feedback topics are rejected in this MVP.

## Arming and firmware behavior

Normal builds and `use_nmpc:=false` select the existing PID path. The NMPC
controller does not inherit the PID allocator and never publishes gimbal_dof,
attitude gains or an allocation matrix. Only one controller plugin is loaded.
The NMPC navigator preserves the existing takeoff/hover/land state machine
but checks its arming gate **before** that state machine can send ARM_ON.
FlightNav roll/pitch/yaw pose targets are supported; the old Gimbalrotor
`final_target_baselink_rot/rpy` moving-CoG mechanism is deliberately absent.
XY velocity-only, local-velocity and attitude-only navigation modes are rejected;
use pose or pose/velocity trajectory targets in POS_CONTROL_MODE.

All profile confirmations must be true and all arrays populated before the
controller initializes. Initialization failure terminates startup before
the navigation timer runs. While disarmed, the controller sends the existing
MotorInfo/UavInfo, obtains a fresh ARM_OFF acknowledgement, and calls
`set_attitude_control(false)`. It requires both success and the exact marker
`nmpc_direct_thrust_v1`; old firmware is not accepted. Missing feedback keeps
arming inhibited. A successful bounded solve precedes the navigation gate.

Firmware direct-mode code is compiled only with **NMPC_DIRECT_THRUST=1**,
STM32H7_V2, DSHOT=0, DYNAMIXEL=1 and KONDO=0. Define NMPC_DIRECT_THRUST
consistently for every C++ translation unit in the STM32CubeIDE project;
the default is zero. Board timer initialization and PWM channel mapping are
unchanged. No firmware compilation or flashing was performed here.

The service accepts changes only when disarmed and PWM testing is off. It
requires four physical DRONE motors, rotor coefficient 1 and gimbal_dof 0.
Boot fresh when changing from legacy gimbal allocation: the patch deliberately
does not silently reinterpret an already initialized eight-virtual-rotor FC.
Mode changes clear stale terms. Direct mode ignores gimbal_dof and PWM-test
requests, freezes the accepted motor conversion profile, bypasses all MCU
attitude/yaw additions and gimbal allocation, and retains the existing PWM
conversion and voltage compensation. Zero thrust explicitly selects IDLE_DUTY;
positive thrust is checked against the current voltage-dependent limit and
PWM output is checked/clamped. All unused timer outputs stay idle.

The direct command watchdog runs even before the first command, has a 100 ms
deadline, and cannot be refreshed by repeated ARM_ON. Wrong lengths, nonfinite
values, nonzero attitude-command fields, timeout, bad conversion or force
landing latch a motor-stop fault. New commands cannot clear that fault.
Existing ARM_OFF semantics and the force-landing acknowledgement are retained,
but **direct-mode emergency action is motor stop, not stabilized descent**.
The host repeatedly requests ARM_OFF on a latched fault and requires restart.
This is a consequential policy requiring explicit owner review before any
arming; `emergency_motor_stop_verified` defaults false. There is no automatic
in-flight PID handover. To return to PID, disarm, restart/power-cycle the FC
and launch the normal PID profile, or explicitly acknowledge the disarmed
attitude-mode service before restarting the host.

The original servo publisher can attach fresh stamps to cached positions.
With the NMPC firmware option, successful CRC-checked Dynamixel position
reads advance a per-servo sequence. The publisher includes only newly read
positions, and excludes positions older than 50 ms before publication.
The host requires every configured servo index to remain fresh independently.
This small driver/bridge-side change is necessary for trustworthy stale-servo
detection; it does not alter Dynamixel commands or ROS message formats.
The default firmware build retains its prior reporting behavior. Actual UART
timing, message loss and the combined acquisition/transport age bound remain
part of propeller-off validation.

## Future native Ubuntu 22.04 setup — not executed here

Use the user's existing edited master_nmpc source checkout on each machine.
Do not assume this local branch is published, replace it with upstream, or
copy x86_64 binaries onto the aarch64 VIM4. Source archives can be transferred;
native libraries, solver/plugin libraries and renderer must match the CPU.

Verify the native OS/CPU and ROS One installation. This repository's README
and `configure.sh` target `/opt/ros/one/setup.bash`; review its jammy-testing
APT channel before installing ROS. `aerial_robot_one.rosinstall` is the
workspace dependency manifest. No ROS environment, vcs import, rosdep or
catkin build was run in this task.

```bash
# FUTURE USER COMMANDS on each native Ubuntu 22.04 machine
uname -m
ls -l /opt/ros/one/setup.bash
source /opt/ros/one/setup.bash
printf '%s\n' "$ROS_DISTRO"  # one

# Set these for your own source checkout/dependency location.
export ROBOT_REPO="$HOME/ros/jsk_aerial_robot"
export ACADOS_SOURCE_DIR="$HOME/acados-v0.5.0"

# On a machine without the dependency sources, choose a NEW external directory:
python3 "$ROBOT_REPO/robots/gimbalrotor/scripts/nmpc/prepare_sources.py" \
  --destination "$ACADOS_SOURCE_DIR"

# Native build prerequisites; review before executing.
sudo apt update
sudo apt install -y build-essential cmake pkg-config python3-dev python3-pip python3-venv cargo

# GENERIC is a conservative native CPU target; benchmark before optimizing it.
cmake -S "$ACADOS_SOURCE_DIR" -B "$ACADOS_SOURCE_DIR/build" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON \
  -DACADOS_INSTALL_DIR="$ACADOS_SOURCE_DIR" \
  -DBLASFEO_TARGET=GENERIC -DHPIPM_TARGET=GENERIC \
  -DACADOS_UNIT_TESTS=OFF -DACADOS_EXAMPLES=OFF
cmake --build "$ACADOS_SOURCE_DIR/build" -j4
cmake --install "$ACADOS_SOURCE_DIR/build"

# v0.0.34 source has no committed Cargo.lock. Keep the generated lockfile
# after the first native build; resolve Rust/dependency compatibility locally.
cargo build --release \
  --manifest-path "$ACADOS_SOURCE_DIR/interfaces/acados_template/tera_renderer/Cargo.toml"
cp "$ACADOS_SOURCE_DIR/interfaces/acados_template/tera_renderer/target/release/t_renderer" \
  "$ACADOS_SOURCE_DIR/bin/t_renderer"

# Python 3.10 on Jammy; separate generator tooling from ROS Python packages.
python3 -m venv "$HOME/.venvs/gimbalrotor-nmpc"
"$HOME/.venvs/gimbalrotor-nmpc/bin/python" -m pip install \
  -r "$ROBOT_REPO/robots/gimbalrotor/scripts/nmpc/requirements.txt" \
  -e "$ACADOS_SOURCE_DIR/interfaces/acados_template"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"

# Sources only: this does not invoke make/CMake or compile a solver.
"$HOME/.venvs/gimbalrotor-nmpc/bin/python" \
  "$ROBOT_REPO/robots/gimbalrotor/scripts/nmpc/gen_nmpc_code.py" \
  -m GimbalrotorTiltQuadServoNMPC

# Later, from your actual catkin workspace with its existing ROS dependencies:
# vcs import src < src/jsk_aerial_robot/aerial_robot_one.rosinstall --recursive
# rosdep install -y -r --from-paths src --ignore-src --rosdistro one
# catkin build gimbalrotor --cmake-args -DGIMBALROTOR_BUILD_NMPC=ON \
#   -DACADOS_ROOT="$ACADOS_SOURCE_DIR"
```

The generator writes inside `robots/gimbalrotor/generated/nmpc`, ignored by
Git. CMake builds `libgimbalrotor_nmpc_solver.so` directly from generated C,
then `libgimbalrotor_nmpc_controller.so`, linked to native libacados, HPIPM,
BLASFEO and the robot model. No Python runs in the ROS control loop. When
the build option is OFF, neither generated files nor acados are required.
Regenerate and rebuild after changing symbolic dynamics, layout, horizon,
sampling time or weights. Runtime checks reject mismatched generated weights,
horizon and period. Physical parameter changes use runtime parameters but
still require renewed owner verification and hardware calibration review.
For a custom profile, pass `--profile /absolute/path/to/your/Nmpc.yaml` to the
generator and the same path as `nmpc_profile` at launch.

After native build and separate firmware validation, make a user-owned copy
of `config/quad/Nmpc.yaml`, fill every physical limit/time-constant array and
confirm each guard only after verification. Reconcile URDF mass/inertia/CoG,
motor spin/order, m_f_rate, MotorInfo PWM/voltage curves, servo index-to-bus-ID
mapping/sign/offset/limits and servo dynamics. The existing values have not
been approved by this task. No Kondo response constant or another robot's
calibration was transplanted.

```bash
# FUTURE only, after build/calibration/firmware/propeller-off validation:
roslaunch gimbalrotor bringup.launch airframe:=quad use_nmpc:=true \
  nmpc_profile:=/absolute/path/to/your/verified/Nmpc.yaml
# Normal selectable fallback, with the FC restored to its normal disarmed mode:
# roslaunch gimbalrotor bringup.launch airframe:=quad use_nmpc:=false
```

Generation/build/API linkage, solver feasibility and timing on VIM4, MCU fault
timing, torque enable and servo command behavior, actuator ordering, arming,
normal landing and emergency motor stop all remain **unverified**. Initial
communication/sign/failure checks must be performed without propellers.
No claim of flight readiness follows from this static review.
