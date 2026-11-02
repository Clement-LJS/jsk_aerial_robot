# Disarmed recurrence and offline reproduction

## What the new bag establishes

Input: `/home/clement/rosbag/year2026/10Oct/261030/inspection/22_yes.bag`.
Recording interval: 2026-10-30 15:00:58.069–15:03:10.633 JST.
The user confirms the robot was disarmed and may have been moved by hand.
It contains 23,815 IMU messages, 6,613 servo-state messages and four ROS logs.

| Recorded time, JST | Observation |
| --- | --- |
| 15:01:16.133 | rosserial_server rejects topicId=162, length=21, for bad checksum |
| 15:01:48.637 | IMU timestamp steps backward by 0.968057 ms; matching warning |
| 15:02:26.336 | One malformed 68-byte ServoStates payload |
| 15:02:27.061 | servo_bridge reports Buffer Overrun for a 68-byte message |

No std::bad_alloc is logged in this recording. The malformed servo payload
declares 65,537,000 records, requiring 458,759,012 serialized bytes rather
than the 68 bytes actually present. Its leading stamp decodes as second 8,
not the current epoch. That bogus servo timestamp belongs to the corrupt
payload; it is not another ordinary clock-synchronization event.

The first 20 bytes are:

```text
08 00 00 00 e8 03 e8 03 e8 03 e8 03 00 00 00 00 00 00 00 00
```

They exactly match a constructed `spinal/Pwms` payload with eight uint16
entries `[1000, 1000, 1000, 1000, 0, 0, 0, 0]`. At servo count offset 8,
the bytes `e8 03 e8 03` become uint32 65,537,000. The schema is verified by
`aerial_robot_nerve/spinal/msg/Pwms.msg:1` and generated `Pwms.h:25–38`.
This is a stronger signature than generic random noise. However, the bag
does not contain `/gimbalrotor/motor_pwms`, so it is a schema-consistent
candidate source, not a matched contemporaneous recorded motor message.
There was no nontrivial IMU-prefix match within +/-1 second of reception.

The existing firmware publishes motor-output telemetry through the same
rosserial mechanism (`attitude_control.cpp:215–218`), so the common outgoing
buffer vulnerability can affect this publisher as well as IMU publication.
No actuator commands were sent to investigate this.

The checksum rejection is a separate recorded event. ROS bags contain no
payload for a serial frame rejected before ROS publication. Topic ID 162 is
negotiated for a rosserial session; this bag lacks the mapping needed to name
its topic confidently. A checksum warning alone does not establish whether
the cause is software interleaving, serial interference, or another defect.

All IMU packets have the expected size, finite values and quaternion norms
within the inspection tolerance. The IMU does show handling motion: in the
four-second window centered on the malformed servo packet, maximum angular
speed is 3.339 rad/s (about 191 degrees/s), median angular speed is 1.013 rad/s,
and acceleration magnitude spans 7.47–13.87 m/s². This is consistent with the
user's report of moving the disarmed robot. It does not establish movement
as a necessary or sufficient trigger for the race.

Reception gaps still reach about 253 ms for IMU and 257 ms for servo data.
The approximately 725 ms difference between reception of the malformed
payload and its error log is a correlation, not a measured allocation time.

## Reproduction levels

1. **Recorded decoding failure:** feeding the saved malformed bytes to the
   inspected Python ROS decoder produces an actual genpy DeserializationError.
   This reproduces an underfilled message, without a ROS node or a robot.
2. **Candidate mechanism:** a deterministic in-memory model interleaves two
   publishers sharing one 1024-byte serialization buffer. It creates valid
   rosserial framing/checksums around invalid servo payloads. The PWM example
   reproduces the same count of 65,537,000. This is not execution of STM32
   firmware or proof of the actual scheduling interleaving on the robot.
3. **Live recurrence:** the fault has already occurred disarmed. A selective
   recorder can capture another occurrence, but does not force the MCU race.
   Adding ROS subscribers does not itself request a higher MCU publish rate.
   There is no demonstrated command that reliably triggers the live fault.

These levels must not be conflated. The offline model also demonstrates
checksum rejection when a byte changes after checksum construction, and a
valid eight-servo packet when complete publish operations do not interleave.
The latter is a control example, not a deployed synchronization fix.

## Run the offline reproduction

From the `robots/gimbalrotor` directory:

```bash
python3 debug/inspection/rosserial_20261030/reproduce_rosserial_offline.py \
  --ros-python-path /opt/ros/one/lib/python3/dist-packages \
  --bag "$HOME/rosbag/year2026/10Oct/261030/inspection/22_yes.bag"
```

Without `--bag`, it runs only the standard-library buffer model, with no ROS
Python dependency. The library-path option selects the existing ROS1 Python
installation; it does not configure ROS networking or start a master.

The run performed during this investigation produced:

- IMU interleaving: valid serial checksum, invalid servo count.
- PWM telemetry interleaving: valid serial checksum, count **65,537,000**.
- Noninterleaved control: valid serial checksum and eight valid servo records.
- Post-checksum byte change: invalid checksum.
- Actual saved payload: `DeserializationError: unpack requires a buffer of 7 bytes`.

Python genpy appends one servo record at a time and fails at the first missing
record. With the 68-byte input limit, at most eight records can decode. It
does not allocate an array of the declared 65 million entries. The script's
own preflight parser uses integer arithmetic only and allocates no servo array.
The original C++ receiver may resize a vector first; that difference is why
this reproduction does not claim to reproduce C++ std::bad_alloc. No deliberate
out-of-memory experiment or malformed-message publishing is included.

The script prints JSON, creates no ROS node, publishes nothing, opens no
serial device, connects to no master, and writes no files itself. The observed
run output was saved separately as `offline_reproduction_result.json`.

## Selective live capture, run manually by the user

From the same directory, with the usual robot stack and ROS environment
already running:

```bash
bash debug/inspection/rosserial_20261030/record_selected_telemetry.sh
```

The script writes to `~/rosbag/year2026/10Oct/261030/inspection`, uses automatic
timestamp filenames, and splits every five minutes. It subscribes only to:

| Topic | Purpose |
| --- | --- |
| `/gimbalrotor/imu` | Raw sensor payloads, embedded time, handling motion |
| `/gimbalrotor/servo/states` | Detect malformed servo packets |
| `/gimbalrotor/motor_pwms` | Compare suspected PWM-telemetry prefixes directly |
| `/gimbalrotor/rpy/pid` | Compare another publisher sharing the outgoing buffer |
| `/gimbalrotor/servo/target_states_debug` | Compare target-state telemetry bytes |
| `/gimbalrotor/servo/torque_states` | Record servo enable-state context |
| `/gimbalrotor/flight_state` | Establish disarmed/armed state in the capture |
| `/rosout` | Correlate checksum, decoding and timestamp warnings |

Equivalent command:

```bash
mkdir -p "$HOME/rosbag/year2026/10Oct/261030/inspection"
cd "$HOME/rosbag/year2026/10Oct/261030/inspection" &&
rosbag record --buffsize=64 --split --duration=5m \
  /gimbalrotor/imu \
  /gimbalrotor/servo/states \
  /gimbalrotor/motor_pwms \
  /gimbalrotor/rpy/pid \
  /gimbalrotor/servo/target_states_debug \
  /gimbalrotor/servo/torque_states \
  /gimbalrotor/flight_state \
  /rosout
```

No `-a`, high-bandwidth camera/TF capture, bag playback, command publisher,
arming, takeoff, or node restart is included. Missing selected topics simply
remain pending. `--buffsize=64` limits the recorder queue, not total process
memory or disk use. Five-minute splitting limits individual file duration,
not total disk consumption; it does not delete older files.

Start with the robot disarmed, since recurrence has already been observed in
that state. Capture a stationary baseline and note times of any ordinary
manual handling. Neither handling nor flight is a proven requirement. Stop
the recorder with Ctrl+C and send the completed split bags covering the event.
The script was checked with `bash -n` but was not started by the assistant.

## Files and constraints

- `bag22_evidence.json`: bag findings, malformed bytes, logs, motion checks.
- `reproduce_rosserial_offline.py`: deterministic model and offline decoding.
- `offline_reproduction_result.json`: output from the executed offline run.
- `record_selected_telemetry.sh`: user-run selective recording wrapper.
- `bag22_reproduction.md`: this report and usage.

The requested offline reproduction was executed; no firmware or ROS runtime
tests were performed. Existing production source and configuration are
unchanged. No builds, commits, pushes, SSH operations, live recording,
actuator commands, or flight operations were performed.
