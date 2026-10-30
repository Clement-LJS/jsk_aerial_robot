PROPELLER-OFF ROSSERIAL BENCH CAPTURE

Purpose
Exercise the existing navigation, allocation, MCU attitude control, gimbal
servos and IMU feedback while capturing the suspected serialization race.
Firmware equivalence to local source is ASSUMED at the user's request; the
ST-Link comparison did not establish it. No firmware fix is included.

Remove ALL propellers. Motors and servos remain real moving hardware. Keep
hands clear of motors, gimbals and linkages; move only the body gently, and
keep the physical power disconnect accessible. This profile is NOT for flight.

Why the launch command changes
The ordinary launch argument is roll_yaw_perching:=true, with airframe:=beetle.
There is no rollyawperching argument. For THIS test, use bench.launch below.
It includes that bringup, applies max_pwm=0.55 (1100 PWM-equivalent),
min_pwm=0.5 (1000), and min_thrust=0 BEFORE the existing controller starts.
It also gates keyboard arm/takeoff through the capture program. A Python
process started after ordinary bringup cannot reliably replace the limits
already cached by the controller. Do not run ordinary bringup simultaneously.

Sequence (use your existing ROS environment; no catkin build needed)
Run from this repository's robots/gimbalrotor directory. Each numbered step
can use a separate terminal. Start with the robot disarmed.

1. Start the bench version of bringup:

   roslaunch simulate_run/bench.launch

2. Set the two mechanism joints to zero, as you proposed:

   rostopic pub -1 /gimbalrotor/joints_ctrl sensor_msgs/JointState "{name: ['joint_roll', 'joint_yaw'], position: [0.0, 0.0]}"

3. Start capture and wait for READY (at least five seconds of baseline):

   python3 simulate_run/bench_capture.py --props-removed

4. Open your normal keyboard controller, with an explicit namespace:

   rosrun aerial_robot_base keyboard_command.py _robot_ns:=/gimbalrotor

   Press r yourself. Wait for flight_state value 2 (ARM_ON) and
   capped_motor_info_observed in the capture terminal. Then press t yourself.
   If takeoff is rejected because initialized PWM feedback has not arrived,
   wait briefly and press t again. Requests are not queued for later execution.
   The script never arms, takes off, or rearms on its own.

5. Follow the capture terminal's four 20-second phases after your takeoff:
   steady; gentle roll/pitch body movement; gentle yaw body movement; steady.
   Hand movement changes the real IMU/estimator input. The repository's real
   controller computes the response and commands its gimbals and motors.
   There are no synthetic IMU messages, duplicate allocation algorithm,
   independent servo sweeps, or PWM-test commands from this Python program.
   It does not initiate a perching maneuver or move the roll/yaw mechanism.
   TAKEOFF need not reach HOVER when the propellers are absent.

6. At 80 seconds, or an anomaly/timeout, the program requests halt and records
   five more seconds. It closes the bag and automatically runs offline analysis.
   Keyboard h remains available throughout; Ctrl-C in capture also requests halt.
   A stop request/observed disarmed state is not physical proof of stopped motors.
   If stop is not confirmed, use h and the physical power disconnect.

An early checksum/deserialization error stops this run intentionally: the
evidence is already captured. There is no automatic repeated arm/takeoff loop.
Restart capture manually, while disarmed, for another independent trial.
Do not increase the ceiling automatically if the motors do not spin.

Where files go
No filename needs to be supplied. Every run creates a timestamped subfolder in:
  $HOME/rosbag/year2026/10Oct/261030/inspection/
It contains automatically named .bag files, events.jsonl, recorder.log and
analysis.json. Send the COMPLETE run subfolder, including all split bags.
Optional destination: --output /your/inspection/folder
The path is on the computer where you run the Python program. All files added
by Codex are local; nothing has been copied to the onboard computer. If bringup
runs onboard, this bench launch must be used there, with the same startup
parameters/remaps. Running only the capture Python beside ordinary bringup
does not impose the motor limit and will fail its checks.

The recorder uses a 64 MiB message buffer, a 128 MiB free-space floor and
five-minute splitting, with explicit topics (never record -a). Python's
256 MiB startup free-space check leaves some headroom. 64 MiB is not a bound
on total process memory. Startup waiting is limited to 180 seconds, armed
waiting for takeoff to 30 seconds, and the active sequence to 80 seconds.
Selected topics include IMU, servo states/targets/torque, PWM telemetry,
motor configuration, flight state/configuration/ack, allocation matrix,
four-axis commands, RPY terms, gimbal/mechanism commands, joint states,
body odometry, battery status, manual/gated commands, /rosout and events.
No video, lidar, point clouds or all-topic recording.

Recorded diagnostics
- Raw AnyMsg payloads are examined BEFORE typed array deserialization.
- Servo count/length checking never allocates from the embedded array count.
- IMU checks include 60-byte length, finite values, stamp validity and quaternion
  norm. Hand-motion magnitudes are not automatically called corruption.
- Backwards source timestamps are counted separately; arrival/source time are
  not treated as synchronized clocks or used to claim one-way latency.
- PWM telemetry must remain at/below 1100, allowing one unit for conversion
  rounding. Empty pre-arming motor arrays are valid; takeoff requires feedback
  from at least four initialized motors after arming.
- A servo anomaly is compared with recent actual IMU and PWM payload prefixes.
  Offline analysis also searches the following one second of recorded messages.
- ROS checksum/deserialization errors, phase changes and stop reasons are saved.
- Prefix matches are evidence of overlapping bytes, not independent proof of
  which task caused an overwrite. No match does not exclude a race.
- The bags contain serialized ROS payload bytes AFTER the rosserial bridge.
  They do not contain complete physical UART frames or checksum-rejected frames.
  No second reader opens the serial port, and ST-Link is not used during capture.

Limits of the low-output configuration
1100 is a PWM-equivalent command, NOT an RPM guarantee. The firmware's normal
conversion path clamps to max_duty after allocation; min_thrust=0 prevents its
voltage update from recalculating a minimum above the ceiling. This changes
the bench motor profile, including the previously observed arming behavior.
The existing control/allocation/servo code remains unchanged.
The cap depends on the assumed firmware, the normal control path, and successful
delivery of the motor configuration. Observing motor_info on ROS is NOT an MCU
acknowledgement. Python's feedback checks/stop requests are additional host-side
checks, not a hard real-time interlock or a guarantee under communication failure.
The program rejects visible PWM-test/alternate gated-command publishers, but
cannot prove the absence of previously enabled firmware test modes or arbitrary
external command sources. Start from the normal disarmed controller setup.
Unexpected loss of Python, ROS or communication can prevent a halt request.
No flight-controller logic, firmware, gains or message rates are changed.

Root-cause assessment motivating this experiment
Strongest explanation: concurrent publishing through the same rosserial
NodeHandle overwrites its shared message_out buffer before UART enqueue.
Earlier Oct26 bag evidence contained exact contemporaneous IMU prefixes in
malformed servo packets. In 22_yes.bag, the corrupt 68-byte packet claims
65,537,000 servo entries (458,759,012 bytes required), and its first 20 bytes
match the FORMAT of an eight-entry PWM payload with four values of 1000 and
four zeros. That bag did not record the matching PWM topic, so the PWM-origin
inference is not yet independently verified. This capture adds that topic.
The user confirmed that 22_yes was disarmed while being moved by hand, so
takeoff/motor operation is not necessary for the fault. No error in one run
does not exclude an intermittent race. This experiment is not a guaranteed
reproducer and does not establish that physical movement causes the failure.

Relevant local source references (relative to repository root)
- aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/ros_lib/ros/node_handle.h:
  publish serializes into shared message_out before hardware_.write.
- aerial_robot_nerve/spinal/src/ros_lib/STM32Hardware.cpp:
  UART queue locking occurs inside write, after serialization.
- aerial_robot_control/include/aerial_robot_control/control/base/base.h:82-85,
  137-153: cached startup motor limits, published during START.
- aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/flight_control/attitude/attitude_control.cpp:
  592-605 receives limits; 1044 recalculates minimum only when min_thrust>0;
  1224-1229 clamps normal output and converts to PWM telemetry units.
  PWM test mode bypasses this normal conversion path; this tool never enables it.
- aerial_robot_control/include/aerial_robot_control/flight_navigation.h:
  enums ARM_OFF=0, START=1, ARM_ON=2, TAKEOFF=3, LAND=4, HOVER=5, STOP=6.

Files added
bench.launch: separate launch with bench parameters and manual-command remaps.
bench_capture.py: user-run recorder, bounded monitor, request gates, phases/stop.
payload_checks.py: bounded raw-payload inspection shared by live/offline tools.
summarize_capture.py: two-pass offline audit including later prefix matches.
test_offline.py: parser and manual-command-gate tests with fake publishers.
README.txt: instructions, rationale and limitations.

Validation performed locally
13 offline tests passed; no ROS node or hardware publisher was run.
The offline analyzer was run against historical 22_yes.bag: it found one
malformed servo payload, the same 65,537,000 count, two relevant ROS log errors,
one backwards IMU stamp and one backwards servo stamp (the corrupt packet).
For historical 50_ins.bag: no malformed servo payload or matching ROS error;
four backwards IMU stamps. Its normal-flight PWM samples exceeding this new
bench ceiling are classified as ceiling violations, not message corruption.
Hardware startup/remapping, low motor output, servos, actual stop behavior and
reproduction rate remain untested. No production files modified; no firmware
build/flash, live ROS execution, commit, push or deployment by Codex.
