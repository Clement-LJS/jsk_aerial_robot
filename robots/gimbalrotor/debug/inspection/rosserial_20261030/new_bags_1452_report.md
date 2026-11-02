# 14:52 recording: arming sequence and communication inspection

The servo-before-motor-output sequence is visible in the recorded telemetry.
The inspected source provides a strong candidate explanation: servo goals and
torque become active immediately on logical arming, while the minimum motor
output derived from minimum thrust is recalculated in a 500 ms periodic block.
Three real backward IMU timestamp steps are also present. The earlier malformed
servo payload / bad_alloc / Buffer Overrun pattern did not recur.

All work was offline and read-only except for inspection artifacts in this
folder. No existing source, configuration, firmware, or hardware was changed.
The user's latest request authorized inspection of arming-related motor/servo
telemetry and source paths; no tuning or actuation was performed.

## Input identity

The supplied paths were renamed during analysis:

- `261030/2.bag` became `261030/2_tuning.bag`.
- `261030/inspection/2026-10-30-14-52-34.bag` became
  `261030/inspection/2_ins.bag`.

The renamed files have the same start/end times and message counts as the
initially opened originals. The active recording also present in that folder
was not read. All times below are JST on 2026-10-30, unless labeled otherwise.

## Arming evidence

The table uses recorder reception times, not exact actuator execution times.
Different topic connections may arrive out of order; in particular, the arm
acknowledgement was recorded slightly before the corresponding command.

| Time | Recorded observation |
| --- | --- |
| 14:52:43.047 | `/teleop_command/start` |
| 14:52:43.068 | START state; motor info carries min_pwm=0.5 and min_thrust=2.0 |
| 14:52:43.069 | ARM_ON command and acknowledgement |
| 14:52:43.082 | `/flight_state` reports ARM_ON |
| About 14:52:43.103 onward | Servo positions change while neutral targets remain in effect |
| 14:52:43.224 | First recorded torque-state sample after arming shows indices 0–3 enabled |
| 14:52:43.080, .178, .278, .388, .487 | Motor-output channels 0–3 remain at 1000 |
| 14:52:43.587 | First higher reported motor outputs: all four channels 1170 |
| 14:52:44.874 | Separate `/teleop_command/takeoff` |
| 14:52:44.883 | TAKEOFF state |
| 14:52:44.909 | Motor outputs increase further and differ between channels |

The first higher motor sample is about 540 ms after the recorded start command,
518 ms after the arm acknowledgement, and 505 ms after the first ARM_ON state.
The preceding output sample still shows 1000 about 100 ms earlier. These are
sampled telemetry intervals, not an exact measurement of the physical spin-up
delay. `/motor_pwms` carries output values without an embedded timestamp, and
no RPM/spin sensor was recorded. The inspected code publishes those values at
roughly 100 ms intervals (`attitude_control.h:76`, `attitude_control.cpp:215`).

The four gimbal target angles remain zero through the arming interval, with
raw target values 2047. Actual angles were already close to neutral before
arming; their subsequent movement is small, not a large full-range sweep in
this particular recording. For example, before arming the measured joint
angles were approximately [-0.0123, 0.0092, -0.0123, 0.0169] rad. At 43.247
they were approximately [-0.0153, 0.0031, -0.0077, -0.0169] rad. Thus the bag
supports early servo engagement/movement but does not prove exact neutral
settling or reconstruct every visible mechanical motion.

See `new_bags_1452_arming.png`.

## Source explanation: why logical arming and motor output differ

Paths below are relative to the Git repository root. No files were edited.

1. `aerial_robot_control/src/flight_navigation.cpp:690–710` sends ARM_ON in
   START state. `include/aerial_robot_control/flight_navigation.h:474–477`
   changes the reported state to ARM_ON when its acknowledgement arrives.
2. `aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/flight_control/flight_control.h:174–192`
   sets the start-control flag and publishes the acknowledgement immediately.
   It does not verify motor RPM or wait for servo-position convergence.
3. `.../flight_control/attitude/attitude_control.cpp:1286–1298` sends gimbal
   goal angles once the start flag is true. `.../servo/servo.cpp:240–275`
   writes the goal and enables torque if needed. The underlying
   `.../servo/drivers/Dynamixel/dynamixel_serial.cpp:149–152` enqueues the
   torque instruction; this call does not itself wait for motion completion.
4. `.../flight_control/attitude/attitude_control.cpp:592–605` loads min_duty
   directly from the received min_pwm, which the bag records as 0.5, and stores
   min_thrust=2.0 separately. A duty of 0.5 becomes the reported output 1000.
5. The same file's `:1005–1047` only recalculates the minimum duty from
   min_thrust within the voltage-update block guarded by a 500 ms interval.
   `:1224` applies that minimum; `:1229` converts the duty to reported units.
6. An offline evaluation of the exact quadratic conversion at `:961, :970–971`
   using recorded min_thrust=2.0, nearby battery voltage 25.193 V, and the
   recorded reference curve gives **1170.984** reported units with rotor
   divider 1. The observed first higher integer value is **1170**. This is
   quantitatively consistent with the minimum-thrust floor becoming active.

The source therefore supports a delay of up to roughly one periodic update
interval, plus reporting/transport timing, before that minimum output is
refreshed. This is a **strongly supported explanation**, not a runtime trace
of min_duty or voltage_update_last_time. The flashed firmware identity remains
unverified. No deliberate servo-settling prerequisite was found in the inspected
arming path. It would be incorrect to label this observation simply as the
previous rosserial byte-corruption problem or as a measured ESC arming delay.

## Payload and timing checks

| Check | Main bag | Diagnostic bag |
| --- | ---: | ---: |
| IMU records | 25,672 | 25,591 |
| Servo-state records | 7,128 | 7,107 |
| Malformed IMU length / nonfinite values / invalid quaternion norm | 0 | 0 |
| Malformed servo count or length | 0 | 0 |
| Nonzero servo error fields / invalid index sequences | 0 | 0 |
| Backward embedded IMU timestamps | 3 | 3 |
| Mean IMU reception rate | 179.97 Hz | 179.99 Hz |
| Mean servo reception rate | 49.99 Hz | 49.99 Hz |
| Largest IMU reception gap | 311.10 ms | 1222.99 ms |
| Largest servo reception gap | 314.89 ms | 313.03 ms |

Every IMU record is 60 bytes and every servo record is 68 bytes with count 8.
All float values checked are finite; quaternion norm deviations are below
approximately 8e-8. Reported servo temperatures span 21–35. No bad_alloc or
Buffer Overrun logs occur. A search of every diagnostic-bag servo payload for
an exact nontrivial IMU prefix (16+ bytes, at least four distinct byte values,
within +/-1 second reception time) found no matches.

Within the common embedded-time interval, trimmed by 20 ms at each end,
25,325 IMU and 7,034 servo payloads match byte-for-byte between the recordings,
including multiplicity. There are no unmatched payloads inside that comparison
interval. This comparison cannot exclude upstream loss shared by both.

There is one 40 ms embedded servo interval at approximately 14:54:44.497,
after the recorded halt. That is twice the usual 20 ms interval; the recording
cannot distinguish a skipped publication from a packet missing upstream.

## New issue: three backward IMU timestamp steps

| Approximate recorded event time | IMU step | Adjacent servo step | Implied common offset change, assuming normal 20 ms servo cadence |
| --- | ---: | ---: | ---: |
| 14:53:00.596 | -1.638777 ms | 13.361223 ms | -6.638777 ms |
| 14:53:53.134 | -1.916465 ms | 13.083535 ms | -6.916465 ms |
| 14:54:03.147 | -1.036520 ms | 13.963480 ms | -6.036520 ms |

Both bags contain the same raw IMU timestamp reversals and matching
`IMU: bad timestamp` warnings from aerial_robot_base_node. These are not just
recorder reception-time irregularities. `/rosout_agg` repeats the warnings;
there are three events, not six.

At each event, the fractional timestamp offset changes identically in IMU and
servo messages. This strongly supports a shared rosserial clock-offset
correction rather than independent corrupt bytes or IMU message reordering.
The local generated node_handle.h requests synchronization periodically at
line 334, applies setNow() at line 370, and changes offsets without a monotonic
guard at lines 384–389. A backward correction of about 6–7 ms can make a
normally 5 ms IMU step negative while leaving a normally 20 ms servo step
positive. Actual sync packets and flashed firmware were not captured.

The host consequence is concrete in local source:
`aerial_robot_estimation/src/sensor/imu.cpp:160–165` logs the warning and returns
from estimateProcess() for a non-increasing stamp. It skips that estimation
update. Other callback processing already occurs before this function, so
this is not a claim that the whole IMU message is discarded by every consumer.
The bag does not establish how much these three skipped updates affect flight.

## Reception stalls: common pauses and one subscriber-specific anomaly

Common IMU/servo reception stalls of approximately 0.25–0.31 seconds remain,
including around 14:53:04.63. Embedded steps across these gaps remain near
5 ms for IMU and 20 ms for servos, supporting backlog/batch reception rather
than equivalent source-sampling pauses. Bag times are ROS reception times,
not verified monotonic time or synchronized one-way latency measurements.

There is also a distinct diagnostic-only IMU gap from **14:54:03.944 to
14:54:05.167**, lasting **1222.99 ms**. The first delayed IMU payload had
already been recorded in the main bag at **14:54:03.951**, about **1215.77 ms
earlier in recorded time**. Its embedded step remains 5 ms. Forty-one IMU
records spanning 222 ms of source time then arrive within 20 ms. Servo
reception in the diagnostic bag does not have a corresponding 1.2 s gap.

This specifically supports a delay on that subscriber/recording path. It is
not evidence of a universal FC/IMU pause lasting 1.2 seconds. No recorder
buffer-exceeded warning is recorded, and both bags eventually contain the
same payloads in their shared interval. Precise attribution among TCP,
subscriber scheduling, and host workload needs another observation.

See `new_bags_1452_timing.png`.

## Artifacts and limits

- `new_bags_1452.json`: raw bad-timestamp packet pairs, numeric checks, clock
  correlations, arming-window samples, comparison counts and conversion check.
- `new_bags_1452_arming.png`: servo positions, reported motor output, and
  reported state around arming.
- `new_bags_1452_timing.png`: reception gaps in both recordings.
- `new_bags_1452_report.md`: this report.

No source fixes, configuration changes, or suggested gain changes were made.
No new flights, live diagnostics, SSH commands, builds, tests, commits, pushes,
or deployments were performed. Analysis used offline bag reads, arithmetic,
and plotting. Physical motor RPM, exact actuator event timing, deployed
firmware identity, and internal minimum-duty update timing remain unverified.
