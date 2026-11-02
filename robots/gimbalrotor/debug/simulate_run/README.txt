PASSIVE CAPTURE FOR NORMAL GIMBALROTOR BRINGUP (LAPTOP OR ONBOARD PC)

Canonical source location is now gimbalrotor/debug/simulate_run. The old
gimbalrotor/simulate_run path remains a compatibility symlink for active runs.
Known VIM4 limitation (2026-11-01): kernel 5.4.125 has CONFIG_USB_MON unset.
The USB commands below apply only on a host with usbmon support; they will not
work on that VIM4 merely by installing tcpdump. ROS topic recording still works.
See ../LIVE_VERIFICATION.txt for the planned alternative raw-byte capture.

Use your normal bringup and keyboard. This recorder never commands the robot,
changes parameters, blocks arming/takeoff, or halts on an error. You control
start, takeoff and stop. The existing controller performs allocation and
servo/motor control in response to your real IMU and hand movement.

ONBOARD PC OVER SSH
USB capture must run on the computer physically connected to the flight
controller USB adapter. SSH is only the terminal connection; it does not make
the remote USB device available to a capture running on your laptop.
For automatic run-folder matching, run BOTH recorders on that onboard computer,
in two SSH terminals, from the SAME recording directory. Both filesets will be
stored on the onboard disk. Keep both SSH sessions open throughout the capture.

First transfer/update this complete simulate_run directory onto the onboard
workspace yourself. Staging files locally does NOT transfer them to the robot.
Use the onboard ROS environment that runs bringup: rospack must find gimbalrotor,
rosbag must be on PATH, and python3 must import rospy and rosbag. USB capture
uses only Python's standard library, tcpdump and kernel usbmon support.
The onboard prerequisites/USB device have not been checked by these instructions.

SSH terminal 1 (ROS capture):

  CAPTURE_CODE="$(rospack find gimbalrotor)/debug/simulate_run"
  mkdir -p "$HOME/rosbag/year2026/11Nov/261101/inspection"
  cd "$HOME/rosbag/year2026/11Nov/261101/inspection"
  python3 "$CAPTURE_CODE/bench_capture.py"

Leave it running. SSH terminal 2 (USB capture):

  CAPTURE_CODE="$(rospack find gimbalrotor)/debug/simulate_run"
  cd "$HOME/rosbag/year2026/11Nov/261101/inspection"
  sudo modprobe usbmon
  sudo python3 "$CAPTURE_CODE/capture_usb.py"

These commands use the SSH user's home directory, not /home/clement. For a
different recording location, change the directory in both terminals equally.
The default adapter serial AU0JZM07 identifies the FTDI adapter used on the
laptop. If you use a different adapter onboard, read its identity with
ls -l /dev/serial/by-id/ and pass its actual serial with capture_usb.py --serial.
Do not launch a second serial bridge or open the tty with another reader.
The USB script discovers the bus/device address; it does not require ttyUSB0.
If the onboard FC connection is not USB through a supported FTDI adapter,
this USB capture setup will need adapting; ROS topic recording remains usable.

CURRENT LOCAL CONNECTION (verified 2026-10-30)
Bridge: /gimbalrotor/rosserial_server, /dev/ttyUSB0, 921600 baud.
USB: FTDI FT232R, serial AU0JZM07, currently bus 3 / device 7.
The device number can change on replug; capture_usb.py discovers it each run.
Live messages were successfully received through ROS. No second tty reader
was opened. Your existing sensors.launch.xml serial-port edit was preserved.
Normal bringup currently has max_pwm=0.85 (1700 PWM-equivalent), min_thrust=2.
The recorder DOES NOT apply the old 1100 limit. --pwm-ceiling is reporting only.
The optional bench.launch is still available for low-output startup settings;
using it includes normal bringup, and its current version has no command remaps.
No existing flight/controller logic is edited by these capture tools.

ROSBAG + USB: COPY THESE COMMANDS, RUN BEFORE ARMING
Keep your normal bringup running. In a second terminal with your ROS environment:

  cd /home/clement/rosbag/year2026/11Nov/261101
  python3 /home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor/debug/simulate_run/bench_capture.py

Leave that terminal running. In a third terminal:

  cd /home/clement/rosbag/year2026/11Nov/261101
  sudo modprobe usbmon
  sudo python3 /home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor/debug/simulate_run/capture_usb.py

No paths or filenames need substituting for this example. On another day, change
only the cd directory in BOTH terminals to your desired existing recording folder.
Do not cd to the source folder: the absolute script path finds the code while
your current working directory determines where recordings are saved.

bench_capture.py creates one timestamped subfolder in the directory where you
run it. There is NO hard-coded October destination. USB automatically finds the
active ROS run in that same parent directory and writes into that run folder.
A process-held file lock distinguishes active runs from old/crashed recordings.
No active run or multiple active runs produces an explanatory error rather than
silently selecting an old folder. Both recorders continue until separately stopped.

Optional overrides: bench_capture.py --output selects a PARENT folder;
capture_usb.py --output selects an existing exact RUN folder. Normally omit both.
Wait until recorder.log says Recording to and the terminal shows streams_received.
This is an observation, not an arm/takeoff gate; your keyboard always works.

Then use your normal keyboard, e.g.:
  rosrun aerial_robot_base keyboard_command.py _robot_ns:=/gimbalrotor

Arm and request takeoff yourself. With all propellers removed, move only the
body gently and keep clear of motors, moving gimbals and linkages. Without
propellers the controller cannot achieve its expected climb and can request
increased thrust. Hand movement also causes corrective servo movement; this
is a control/communication workload test, not physical hovering.
Stop/disarm with your keyboard when done, then Ctrl-C the recorder.
Stopping/crashing this recorder DOES NOT stop/disarm the robot.
Errors are evidence and do not stop recording. Optional --duration 120 stops
only recording after 120 wall-clock seconds. No automatic arming or rearming.

WHAT THE TOPICS AND EXTRA FILES TELL US
- IMU + servo states: raw lengths/counts, timestamps, exact mixed-message bytes,
  and whether the body was stationary or moving near the failure.
- PWM + ESC telemetry + battery: output/electrical context and candidate PWM
  payloads which might have overwritten servo bytes. ESC telemetry may be silent
  when not enabled; no driver/configuration change is made to enable it.
- flight state, arm/takeoff/halt, configuration and ACK: which command/state
  transition preceded the error. 16=LOW_BATTERY and 17=FORCE_LANDING are valid
  status overrides; the underlying navigation state is not separately encoded.
- four-axis commands, allocation matrix, RPY terms/gains, target forces, pose PID:
  whether control/allocation was active and its traffic around an error.
- pwm_test: direct PWM tests, including tests performed outside normal arming.
- actual servo target/current/torque commands and target feedback: command versus
  feedback traffic, not just a host-side joint-state reconstruction.
- mocap/body odometry and perching status: estimator/mode context.
- /rosout: checksum rejections, deserialization errors, serial reconnects and
  other node logs. A rejected serial packet never appears as an IMU/servo topic.

The explicit topic list is TOPICS in bench_capture.py. There is no record -a,
video, lidar or point-cloud recording. The message buffer is 64 MiB, with
five-minute bag splitting and a free-disk check. 64 MiB is not total process RAM.
Some topics publish only during arming/control. motor_info being not-yet-observed
before arming is normal, and will not block or stop anything.

Files in each run folder:
- Automatically named .bag files: serialized topic payloads, including malformed
  servo payloads because capture uses raw/AnyMsg rather than typed array decode.
- events.jsonl: errors, state changes, manual commands, rates, bounded raw samples,
  PWM/IMU prefix overlap and one-second pre-error motion/traffic context.
- analysis.json: final offline analysis; for each retained error/anomaly it adds
  one-second before/after topic counts, gyro/acceleration ranges, recent manual
  commands, flight state and PWM. Matching also searches later recorded samples.
- metadata_start.json / metadata_end.json: selected ROS parameters, selected graph
  connections/types, node PIDs/URIs, local git revision/status, USB identity and a
  bounded excerpt of relevant existing local bridge logs. No all-parameter dump.
- connections.json: actual publisher caller IDs, message types and MD5s observed.
- recorder.log: recorder startup, disk/buffer/drop errors and finalization output.
- .ros_capture.lock: run discovery only; an unlocked file does not mean active.

Source timestamps, local monotonic arrivals and bag receipt times remain separate.
Do not infer one-way latency between unsynchronized computers. Detailed live logs
are capped at 100 per category; counters and ordinary bag recording continue.
History/diagnostic queues are bounded; any diagnostic queue drops are counted.

OPTIONAL USB BYTES: NEEDED TO SEE CHECKSUM-REJECTED FRAMES
Topics alone cannot expose the bytes the bridge rejects. A passive USB trace
captures host USB transfer data in both directions without opening ttyUSB0 or
interfering with ownership of the serial port. It is not an electrical UART
waveform or a trace of MCU task execution. The November 1 laptop capture was
validated offline: 186,342 USB records, zero reported kernel drops, and valid
reconstructed serial frames. This does not verify onboard kernel/adapter support.

Use the third-terminal commands above, after starting the ROS recorder. No run
path needs copying. The script creates its own timestamped filenames in the
automatically discovered run folder. It uses installed tcpdump, selects the
FTDI device by serial AU0JZM07, and filters bulk transfers to that device only.
It does not capture the camera, Bluetooth or other USB devices. It does not
open the serial port, change its baud rate, reset/reflash the FC, load modules
itself, or change existing node log levels. The separate modprobe command loads
Linux's passive tracing facility; it is not a firmware operation.

Look in the printed USB .log for tcpdump's listening line. Run the same manual
trial. Stop/disarm the robot yourself, then Ctrl-C both capture terminals.
Send the full run folder, including USB .pcap/.json/.log and ROS files. The USB
log's final capture/drop counters matter. A replug/disconnect stops USB tracing;
restart it to rediscover the new device address. There can be a short detection
interval after replug; do not reuse the old trace across USB reconnections.

The PCAP includes FTDI USB status bytes as well as serial data. Later analysis
must remove FTDI packet status headers before reconstructing rosserial frames.
Do not treat all USB payload bytes as serial bytes or infer MCU causality solely
from the PCAP. It can show whether bad checksums/payload bytes were already
present when the USB driver received them and whether transfer errors occurred.

References for the USB method/filter (device address is header byte 11):
https://docs.kernel.org/usb/usbmon.html
https://www.tcpdump.org/linktypes/LINKTYPE_USB_LINUX_MMAPPED.html

REPRODUCTION PLAN: VARY ONE CONDITION AT A TIME
Start recording before manual commands and, if using USB, before arming.
Try short independent phases with steady intervals between them:
1. Disarmed and stationary.
2. Disarmed with gentle hand movement.
3. Armed and stationary (if you choose to arm).
4. Takeoff requested, body steady, then gentle hand movement.
5. Manual stop/disarm, then a final steady recording interval.
Repeat to measure frequency; do not increase PWM or command/message rates just
to force an error. Existing disarmed/stationary captures already contain errors,
so actual flight is not a prerequisite. No deterministic trigger has been proven.
The event context will distinguish motion/state transitions from background
occurrences. Exact RTOS interleaving would still require firmware instrumentation
or a verified matching debug image; the tools do not claim to provide that.

WHY THE EARLIER RUN FAILED / CURRENT FINDINGS
Run 19-06-36 recorded two checksum rejections before your arm/takeoff. The old
script then issued 50 halt commands during cleanup, preventing your test. That
control behavior has been removed entirely. The later passive live verification
also uncovered a monitor bug: state 17 was incorrectly considered malformed.
It is the valid FORCE_LANDING override; this is now recognized and tested.
During the local verification the bridge's recent log contained checksum errors,
and the controller reported FORCE_LANDING. These are observations; the recorder
did not command that state. A clean 15-second sample does not exclude a rare race.

The leading corruption hypothesis remains shared rosserial message_out writes
from concurrent MCU publishers. Previous exact IMU prefixes inside corrupted
servo packets support it. Checksum errors alone can also have other causes;
USB bytes and configuration/state context help narrow the possibilities.
No firmware/source equivalence was established by ST-Link; it remains assumed
at your request. No firmware fix, flashing, control changes, commit or push.
