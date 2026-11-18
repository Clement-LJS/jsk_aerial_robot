# Recorded-command bench replay — 2026-11-18

| Item | Detail |
| --- | --- |
| Author | OpenAI Codex, implementing the user's request |
| Date | 2026-11-18, Asia/Tokyo |
| Target | STM32H7_v2 / Spinal 4.1; recorded four-motor, one-DoF gimbal beetle configuration |
| Files delivered | `replay_commands.py` and this report |
| Changes made | New local replay utility and documentation only; existing firmware, controller and rosserial sources were not modified by this task. |
| Hardware execution | None performed by the assistant. No serial device opened. |
| Git operations | No commit or push. |

## Purpose

Replay the commands that the computer sent to Spinal during the November 17 flight, while Spinal reads its **current physical IMU and servo feedback**. This provides a repeatable command workload for stationary, propeller-off debugging of the firmware and rosserial communication.

This is a **command replay into real hardware**, not a flight simulator. The recorded high-level controller outputs are replayed at the computer-to-Spinal boundary. The old navigation/admittance algorithms are not re-executed. There is no synthetic IMU, position, contact force or cutting resistance.

The same recorded commands combined with a stationary body's live sensor readings will produce different low-level control errors, PID values, motor outputs and gimbal responses than the moving flight did. Motors and servos can move even though the body is held stationary.

## Exact source recording

The default and fully inspected replay source is **one bag only**:

```text
/home/clement/rosbag/year2026/11Nov/261117/5cutattempted_autoscriptfailed_manualcut.bag
```

| Property | Value |
| --- | --- |
| Start | 2026-11-17 11:34:51.646352 JST |
| End | Approximately 11:42:48.261761 JST |
| Duration | 476.615409 seconds, about 7 minutes 57 seconds |
| SHA-256 | `375a93594431a05bb01ccc398a05eef27f7466e90395b8de539adbbc4e60b5bf` |
| Selected command messages | 79,539 |

No `.orig.bag` or inspection split is combined with this bag. This avoids replaying overlapping recordings as duplicate commands. Bags `1adm_failed.bag` and `2aftercodex_tarestillfailed.bag` were inspected to understand initialization and command formats, but are not mixed into this replay. `--bag /absolute/path.bag` can select another single recording that passes the same schema/configuration checks; its runtime results have not been validated on hardware either.

The script needs access to the source bag at execution time. The bag is not copied or embedded into the script. If running on a different computer, supply the appropriate local path with `--bag`.

## What is replayed

Names below are relative to `/gimbalrotor/`. The script reads that source namespace and publishes to `/gimbalrotor` unless `--namespace` is explicitly supplied.

| Topic | Messages in flight 5 | Handling |
| --- | ---: | --- |
| `gimbal_dof` | 17,073 | Recorded value; must be 1 for this implementation. |
| `uav_info` | 1 | Recorded four-motor DRONE configuration. Together with one gimbal DoF, Spinal allocates eight vectoring terms. |
| `motor_info` | 1 | Recorded conversion tables, with the bench motor-limit modifications below. |
| `joint_profiles` | 1 | Recorded joint identities, scales, signs and limits. |
| `rpy/gain` | 3 | Recorded attitude gains. |
| `torque_allocation_matrix_inv` | 16,828 | Recorded allocation matrix rows. |
| `four_axes/command` | 16,828 | Recorded roll/pitch fields, yaw term and eight base-thrust/vector components. |
| `desire_coordinate` | 14,399 | Recorded coordinate rotations; all three components were zero in this bag. |
| `servo/target_states` | 14,399 | Recorded explicit joint targets. This bag commands IDs 7 and 6. |
| `flight_config_cmd` | 3 | ARM_ON, integration-enable and ARM_OFF. ARM_ON/OFF require a fresh live acknowledgement. |
| `pwm_test` | 3 | Preserve the command times and channel, but replace each value with **0.5/off**. |

The allowlist additionally supports `servo/torque_enable` and `perching/servo_neutral_mode` if present in another compatible bag. These topics have no recorded messages in the default bag. The script uses them for explicit initialization/cleanup as described below.

The script does **not** publish recorded `imu`, `servo/states`, `motor_pwms`, `rpy/pid`, `flight_state`, odometry, mocap, acknowledgements or `/clock`. In particular, `motor_pwms` is output telemetry, not the flight-command input. Publishing old telemetry would conceal the behavior being investigated.

It also does not replay operator/navigation topics such as `uav/nav`, `joints_ctrl`, perching motion or admittance-enable commands. Their resulting Spinal-bound commands are already in the selected stream. Replaying both layers would create competing commands and would not faithfully rerun the old closed-loop navigation on a stationary robot.

## Deliberate differences from the original flight

1. **Motor output limit:** every `motor_info` publication uses `min_pwm=0.5`, `max_pwm<=0.55` and `min_thrust=0`. The maximum is 1100 PWM-equivalent under this firmware's `duty × 2000` convention; this is not an RPM guarantee. Clearing `min_thrust` prevents voltage compensation from raising the minimum duty beyond the bench cap. Original flight values were approximately `max_pwm=0.85`, `min_thrust=2.0`.
2. **Saw is kept off:** the recorded `pwm_test` messages target channel 4. Its original 0.8 command would energize the external device. This firmware's external-output path can bypass the ordinary flight-motor clamp, so replaying it unchanged would defeat the bench limit. The replay always substitutes 0.5, with no option to enable the saw.
3. **Initialization is supplied before the replay clock starts:** disarm and confirm; then send recorded gimbal DoF, UAV configuration, capped motor information, joint profiles and gains in a controlled order. The recorded bag's near-simultaneous startup entries place ARM_ON before some configuration messages, which is unsuitable for initializing a fresh board by receipt-order replay alone.
4. **Known initial modes:** explicitly set servo-neutral mode to false and desired-coordinate rotation to zero. Neutral-mode transitions were not recorded in this bag. These are stated initialization assumptions, not reconstructed evidence of every internal firmware setting.
5. **Command order/timing:** use each message's bag-receipt offset from bag start at 1× speed, scheduled using monotonic wall time. ROS/TCP/serial transport and acknowledgement delays prevent a guarantee of identical MCU arrival times or task interleaving. The replay stops if it falls more than 250 ms behind rather than sending a large catch-up burst.
6. **Stopping:** end-of-run, Ctrl-C and detected errors trigger best-effort ARM_OFF, integration-off, saw-off/test-mode-exit and torque-disable for servo IDs 0–7. This is additional cleanup, not an exact replay of the flight's final servo-torque state. The body and mechanism must be supported when servo torque is released.

Except for the motor-info and saw-value transformations, scheduled command payloads retain the recorded fields. The initialization and cleanup add extra messages. This is therefore not a byte-for-byte replay of the entire original flight traffic.

## Run instructions

Use the existing ROS workspace environment and a system Python that can import `rosbag`, `rospy` and the generated `spinal` messages. The script has no custom Python helper dependencies. The bag and ROS installation remain external prerequisites.

### 1. Offline preview — no robot connection

```bash
source /home/clement/ros/jsk_aerial_robot_ws/devel/setup.bash
python3 /home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor/debug/reproduce_261118/replay_commands.py
```

This reads and validates the source commands, prints the source hash/counts and planned modifications, and exits. It does not start a ROS node or create replay outputs. If `python3` selects an incompatible Conda environment, use `/usr/bin/python3` with the same sourced workspace.

### 2. Prepare the stationary hardware session

- Remove all propellers, secure/support the body and mechanism, and keep clear of moving servos and motor shafts. The script is not a physical restraint or emergency stop; keep hardware power isolation accessible.
- Use a fresh, disarmed board session where practical. Configuration and internal state inherited from a previous run can affect reproducibility.
- Start a ROS master if one is not already available, using `roscore` in another terminal. Use the intended `ROS_MASTER_URI`; the script does not start or change the master.
- Do **not** run normal robot bringup/controller/navigation or `servo_bridge` alongside this replay. Their command publishers would compete with the recorded stream. The script checks for such publishers and refuses to proceed, even if a competing publisher is currently quiet.
- The computer running `--start-bridge` must have the physical serial adapter and the correct `/dev/flight_controller` device alias. Override it with `--port /dev/ttyUSB0` if that is the verified device on your machine.

The firmware's onboard IMU and local attitude controller work without the high-level controller node. A computer-side state estimator is not required for this direct command replay. This setup intentionally does not produce the normal high-level odometry/admittance outputs.

### 3. One replay command, including bridge startup and live recording

With the ROS master running and normal bringup stopped:

```bash
python3 /home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor/debug/reproduce_261118/replay_commands.py --execute --props-removed --start-bridge --output /home/clement/rosbag/year2026/11Nov/261118/reproduction
```

The source bag defaults to flight 5. This command starts only `spinal/bridge.launch`, waits for its negotiated command subscriptions and fresh live eight-servo/IMU telemetry, gives a five-second countdown, starts recording and initializes the board. It then replays the full command timeline.

If the serial bridge is already running **by itself**, omit `--start-bridge`. The script never starts a second bridge on an existing node. Do not add a separate `bench_capture.py` merely to get a live bag: recording is already built in.

For a shorter first check, append:

```text
--duration 15
```

For the first original warning window, use `--duration 210`; for both warning/overrun windows, use the default full run. Duration is measured from the source bag start, after initialization, not from the instant you launched Python. There is no accelerated playback or mid-flight seek, so initialization and the preceding command history are retained.

## Checks during execution

- Validate command wire lengths/counts before deserializing and reject unsupported message types, schema MD5 mismatches, nonfinite values and unexpected robot configuration.
- Require live `/use_sim_time=false` and telemetry from the expected rosserial bridge node.
- Reject competing publishers on the selected command topics; recheck the ROS graph during replay.
- Check fresh ARM_ON/ARM_OFF acknowledgements. Unexpected disarm or force-landing acknowledgements abort the replay; firmware failsafes are not disabled.
- Abort on stale IMU, servo or PWM telemetry, a stopped recorder, low disk space, or excessive scheduling delay.
- Watch physical motor telemetry channels 0–3 for values above 1105 PWM-equivalent and saw channel 4 above 1005; abort if exceeded. The small tolerance permits numeric rounding. These are software observations, not independent electrical output enforcement.
- Log malformed servo/IMU/PWM payloads without blindly deserializing their array counts. An individual corrupt packet is retained as evidence rather than immediately ending the experiment; sustained loss of valid telemetry causes a stop.

Cleanup is best effort. USB loss, process SIGKILL, power failure or a firmware hang may prevent a stop command being delivered. The console reports whether a fresh ARM_OFF acknowledgement was received. If it is not confirmed, use hardware power isolation. Acknowledgement confirms the firmware response, not independent measurement of every physical actuator.

The script stops only the bridge and recorder processes it started. It does not terminate existing robot nodes or restore the old high-power motor configuration afterward. The bench cap remains until another configuration update/restart changes it.

## Output from each actual execution

Under the selected output parent, a new `replay-YYYYMMDD-HHMMSS-.../` folder contains:

| File | Contents |
| --- | --- |
| `run.json` | Source bag/hash, source and scheduled counts, replay settings, actual published counts, script hash, and completion/failure information. |
| `events.jsonl` | Actual published payload bytes in hex, replay offsets, phase (initialization/replay/cleanup), acknowledgements, communication anomalies and selected ROS logs. |
| `live_*.bag` | Fresh command traffic and Spinal telemetry, split every five minutes. Includes IMU, servo feedback, PWM, PID, target feedback, torque states, battery, acknowledgements and `/rosout`. |
| `recorder.log` | rosbag recorder output. |
| `bridge.log` | Serial bridge output when this script started it. |

The live bags contain ROS messages after the bridge, not raw UART frames. If the new receiver validator rejects malformed ServoStates before forwarding, the malformed payload will not appear in the ROS bag; the receiver's warning can appear in `/rosout`/`bridge.log` instead. The script does not inject known bad telemetry or force a firmware race.

## Original timeline and useful observation points

Offsets are relative to the default source bag's start. The replay has a separate initialization/countdown before this clock begins.

| Offset | Recorded occurrence |
| ---: | --- |
| 2.252 s | ARM_ON request. |
| 3.589 s | First `four_axes/command`. |
| 4.187 s | Integration enable. |
| 43.206 s | First explicit servo target command. |
| 160.108 s | Saw channel 4 off command. |
| 165.254 s | Original saw activation at 0.8; replay substitutes 0.5/off. |
| Approximately 183.5 s | Original malformed servo packet and false gimbal-1 voltage/error report. |
| 391.481 s | Saw off. |
| Approximately 429.5 s | Original malformed servo packet associated with Buffer Overrun. |
| 473.833 s | ARM_OFF request. |
| 476.615 s | End of recording. |

Recorded explicit servo-target ranges were ID 7: 2024–2053, and ID 6: 2047–2420 raw units. Low-level gimbal targets for other channels are calculated by Spinal from the commands and live IMU; they are not replayed from `servo/target_states_debug`.

The separate console error at Unix timestamp `1794883627.787016709` occurred at 11:47:07.787 JST, after this bag ended. Its exact command history is not available in this selected recording, so this utility does not claim to reconstruct that later instant.

## Interpretation and expectations

The earlier diagnosis found cross-topic PID/PWM bytes inside servo packets and an unprotected shared firmware serialization buffer. Sending the recorded command workload while Spinal publishes live IMU/servo/PID/PWM data is a useful way to exercise that path. It does not guarantee the same RTOS interleaving, physical current draw or error time.

With the repaired publication mutex actually flashed and the rebuilt receiver running, the expected result is that mixed packets no longer occur. A clean run is evidence for that workload and duration, not proof that every possible communication failure is fixed. This tool does not install old firmware or remove the repair to manufacture an error.

Holding the robot stationary, limiting motor output and leaving the saw off deliberately changes physical load and vibration. This cannot reproduce cutting resistance, validate the original admittance performance, or establish an electrical servo-rail root cause. Those require separate controlled physical observations. The original high-level commands are preserved at the firmware boundary, but the original closed-loop flight is not.

## Validation performed during development

- Parsed the complete default bag and validated all **79,539** selected commands.
- Checked the motor cap and saw-off transformation, verified the source messages remain unmodified, and checked bounded rejection of corrupt array lengths and competing command publishers.
- Checked that over-limit PWM telemetry requests an abort, malformed PWM payloads are logged without array deserialization, and an interruption flag immediately ends an interruptible wait.
- Executed an eight-second replay against a **temporary localhost-only ROS master and simulated Spinal node**, using real ROS message classes. Confirmed command delivery, fresh synthetic telemetry, ARM acknowledgements, automatic live rosbag recording and acknowledged disarm cleanup.
- No serial device, physical robot or live robot ROS master was used for these tests. The simulated-node test does not establish physical timing, firmware behavior or safety of a particular mounting arrangement.
- Temporary test scripts, test recordings and test ROS logs were removed. No test files are included in this folder.
