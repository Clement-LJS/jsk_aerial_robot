# Rosserial corruption inspection — 2026-10-30

Existing source logic is unchanged. This folder contains inspection artifacts
only. No fix is installed or claimed. Branch:
`develop/pochitab_passiveperching/main`, HEAD:
`e01f640d347a3e721e2a8a121945337da0047143`.

## Findings and confidence

1. **Confirmed packet corruption:** five recorded 68-byte ServoStates payloads
   have inconsistent array counts. Four contain exact prefixes of nearby IMU
   payloads. This reproduces the supplied forensic claim using
   `261026/21navigation_failed.bag`, without relying on the differently named
   `21navigation_failed(1).bag` in the task description.
2. **Confirmed concurrency vulnerability in the inspected H7/H7_v2 source and
   generated headers:** two enabled tasks use the same node handle and its
   unprotected serialization buffer. **High-confidence candidate cause of the
   recorded contamination**, but the exact deployed binary and task interleaving
   have not been observed.
3. **UART DMA premature slot reuse is not established.** Advancing the consumer
   index before DMA completion looks suspicious in isolation, but the ring's
   reserved slot protects the active transfer in the normal successful path.
   Error/completion handling still warrants verification.
4. **Schema mismatch or a 68-byte buffer limit is not supported by this evidence.**
   Bag definitions match checked-in definitions, and the node's output buffer
   is 1024 bytes. A corrupted array count, not the physical payload length,
   explains the decoding/allocation errors. This does not certify every
   deployed generated header or every other message type.
5. **Receiver-only corruption is strongly disfavored:** raw malformed payloads
   are in the bag independently of the typed servo_bridge subscriber. The
   rosserial bridge itself and the physical UART bytes have not been traced.

No causal claim is made about the navigation/perching failure itself.

## Bag evidence

Read-only Python rosbag file access selected only `/gimbalrotor/imu`,
`/gimbalrotor/servo/states`, `/rosout`, and `/rosout_agg`. There was no playback,
ROS node, environment setup, or ROS-master connection. The installed rosbag
library was imported for offline file reading only.

| Bag (under `~/rosbag/year2026/10Oct`) | IMU records | Servo records | Inconsistent servo counts |
| --- | ---: | ---: | ---: |
| `261029/7gpttuned_2.bag` | 9930 | 2756 | 0 |
| `261029/12.bag` | 3080 | 854 | 0 |
| `261029/5.bag` | 11105 | 3083 | 0 |
| `261026/20perching_failed.bag` | 8546 | 2317 | 0 |
| `261026/21navigation_failed.bag` | 8229 | 2268 | 5 |

All selected IMU records were 60 bytes; all servo records were 68 bytes.
IMU checks found no nonfinite floats, invalid nanosecond fields, or quaternion
norms outside 0.9–1.1. These limited checks do not establish complete sensor
accuracy, absence of arrival gaps, or physical plausibility during all motion.

The five corrupt counts, in recording order, were 3199023753, 3202392497,
58356, 1050683330, and 3197955730. Matching IMU-prefix lengths were 19, 50,
none, 23, and 60 bytes. All matches begin at servo-payload offset zero;
the first IMU acceleration float therefore occupies the servo count field.
Matching used a +/-1 second bag-time window, at least 16 identical consecutive
bytes, and at least four distinct byte values. The four matches were less than
1 ms apart in bag reception time. The unmatched packet is not assigned
an IMU origin without evidence.

`/rosout` contains four `std::bad_alloc` exceptions and one `Buffer Overrun`,
all severity ERROR (8), from `/gimbalrotor/servo_bridge`, matching the five
packets. `/rosout_agg` contains duplicates, not five additional failures.
Exact raw payloads and integer reception timestamps are in `evidence.json`.
Source log stamps, embedded MCU stamps, and reception times are kept separate;
none is treated as a cross-machine latency measurement.

## Source trace

Paths below are relative to the Git root, two levels above `gimbalrotor`.
Line references describe the checkout inspected on this date.

| Stage | Evidence |
| --- | --- |
| One H7_v2 node handle | `aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/Src/main.cpp:114` |
| IMU estimator receives `&nh_` | Same file, lines 268/274 |
| Direct servo receives `&nh_` | Same file, line 281 |
| Separate realtime tasks | Same file, lines 340 and 364 |
| Core task updates estimator | Same file, line 1231 |
| Servo task updates direct-servo object | Same file, line 1371 |
| Forwarding to attitude publisher | `aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/state_estimate/state_estimate.h:63` and `:90` |
| Advertise and publish IMU | `aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/state_estimate/attitude/attitude_estimate.h:99`, `:136`, `:176` |
| Advertise and publish servo telemetry | `aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/servo.cpp:35`, `:156`, `:178`, `:200` |
| Virtual publish dispatch | `aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/ros_lib/ros/publisher.h:53` |
| Shared output buffer | Same board's `ros_lib/ros/node_handle.h:121` |
| Serialization, header, checksum, write | Same generated header, lines 476–508 |
| Integration wrapper lacks serialization override/lock | `aerial_robot_nerve/spinal/src/ros_lib/ros/node_handle_stm.h:50` |
| Queue lock begins after serialization | `aerial_robot_nerve/spinal/src/ros_lib/STM32Hardware.cpp:214`, `:220` |
| Queue copies source bytes | Same file, lines 242 and 251 |
| UART transfer and index advancement | Same file, lines 175–202 |
| Typed host subscriber | `aerial_robot_model/src/servo_bridge/servo_bridge.cpp:130` and `:382` |

H7 follows the same direct-servo structure: global node handle at main.cpp:104,
estimator initialization at 252/254, servo initialization at 261, tasks at
320/344, estimator update at 1098, and servo update at 1238.
H7 and H7_v2 checked-in config headers enable SERVO_FLAG at line 50; F7's
corresponding config disables it. F7 has a CAN/Spine telemetry path in its core
task, so the H7 direct-servo two-task argument must not be blindly applied to
F7. The CAN servo telemetry publisher is in
`aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/Spine/spine.cpp:265–279`.

H7_v2 FreeRTOSConfig.h:58 enables preemption. Its FreeRTOS source
`include/FreeRTOS.h:777–778` defaults time slicing to enabled; tasks.c:2779
implements the equal-priority scheduling condition. Equal realtime priorities
therefore do not serialize the two publishing tasks.

All three local generated node_handle.h copies have SHA-256
`7f92184af328db415c2e11fea21fec5bf9bc27b7e640d5f59ce8ca4c37cbf0f6`.
The inspected H7_v2 generated node_handle_stm.h and STM32Hardware.cpp match
their persistent integration sources. The generated H7_v2 hardware header
enables RTOS support. Locally present build artifacts do not prove which board
or firmware is physically deployed; generated-header and ELF timestamps also
differ. No firmware binary was executed.

## An interleaving that explains the bytes

1. The servo task starts serializing into shared `message_out + 7`.
2. Scheduling switches to the IMU task, which serializes its 60-byte payload
   into the same memory and enqueues its own frame.
3. The servo task resumes partway through serialization, overwriting the
   remainder with servo fields. Alternatively, it had finished serialization
   before the switch, allowing all 60 IMU bytes to remain.
4. The servo task writes its own 68-byte length/topic header and computes a
   checksum over the now-mixed payload, then enqueues that frame.

This permits a correctly framed/checksummed serial packet with the wrong ROS
payload. The observed partial prefixes and full 60-byte prefix fit this
sequence. It is a possible execution of the inspected code, not a captured
runtime trace. Bag records do not contain the original UART framing/checksum.

## Mutex and DMA review

`rosPubMutex` is created before the scheduler starts (H7_v2 main.cpp:295–296)
and its pointer is passed to UART hardware initialization (main.cpp:1208).
It is taken in hardware.write() only while checking queue capacity/copying
bytes and in hardware.publish() while advancing the consumer index. Neither
call protects earlier serialization. The DMA completion semaphore is created
with one token (main.cpp:311–313); publish() waits before starting a transfer.

CMSIS `osMutexCreate` uses a nonrecursive FreeRTOS mutex
(`.../CMSIS_RTOS/cmsis_os.c:597–612`). The generated publish() calls logerror()
on an oversize frame (node_handle.h:507), and logging calls virtual publish()
again (line 522). Reusing the TX mutex around parent publish() would also
reenter that same mutex inside hardware.write(). Either is a deadlock concern.
A future serialization lock must be separate from the queue lock and account
for same-task logging recursion, initialization/failure paths, and ISR callers.
No complete all-publisher/ISR audit or safe timing bound is claimed yet.

In normal successful UART operation, the consumer pointer advances to the next
slot after starting DMA. Enqueue rejects writes that would reach the consumer
pointer, leaving one unused slot: its predecessor, which contains the current
DMA transfer. Before the consumer advances again, it waits for completion of
the previous transfer. Under these assumptions, the active slot remains
protected even at wraparound. **Index advancement alone does not prove a
DMA lifetime bug.**

Remaining queue concerns: completion/error callbacks do not filter the UART
handle (STM32Hardware.cpp:33–43); the error callback releases the semaphore
for DMA errors without establishing TX completion; HAL transfer failure still
advances the index; overflow silently drops a frame; some queue-index reads
occur outside the mutex. Which of these occurs in this deployment is unknown.
No such event was inferred solely from the observed payloads.

## Bounds, schema, and persistent correction location

The checked-in schema gives `8 + 13*4 = 60` IMU bytes and
`8 + 4 + 8*7 = 68` servo bytes. The generated ServoStates serializer writes
the uint32 count at offset 8 (`ros_lib/spinal/ServoStates.h:43–47`). The
direct-servo code allocates its state array with getServoNum() and iterates
that count (servo.cpp:43–46 and :166). No count-derived overflow was established
there; stability of the device count over runtime remains an assumption.

`ros.h:43` configures OUTPUT_SIZE=1024. With rosserial overhead the two frames
are 68 and 76 bytes respectively. UART's 64-byte slots are chunks, not a
message-size limit: write() splits frames across slots. The upstream size
check occurs after serialization, so it is not a general pre-write bounds
guarantee, but these valid message sizes do not approach that limit.

A potential persistent correction belongs in the integration wrapper
`src/ros_lib/ros/node_handle_stm.h`, overriding virtual
`publish(int, const Msg*)` to protect the entire parent operation, while
preserving the existing no-argument UART-drain overload. This is a proposal,
not a completed or verified fix. `scripts/make_libraries.py:93–106` copies
upstream headers, overlays this repository's integration files, and enables
RTOS support. Editing only generated board headers would be overwritten.

## SSH status and remaining work

The user confirmed the configured robot SSH target. It responds to SSH, but
rejects both available agent keys; BatchMode authentication returns
`Permission denied (publickey,password)`. No remote command ran. Network
access required sandbox escalation, which was allowed; the outstanding
failure is SSH authentication, not an automatic approval rejection.
No keys, known_hosts entries, SSH configuration, or remote files were changed.

Actual onboard rosserial version, process configuration, board identity,
flashed firmware, and existing onboard logs remain unverified. Authentication
must be made available, or the user can supply read-only outputs from their
existing onboard terminal. Passwords/private keys should not be placed here.
Even successful SSH would not alone establish the MCU's flashed binary.

Once authentication is available, continue only the approved read-only
deployment inspection. Any production-source correction remains pending the
user's permission to change source logic. A live AnyMsg monitor remains future
work; none was started. Any later recording must use the user-specified
`~/rosbag/year2026/10Oct/261030/` destination.

## Artifacts and verification limits

- `README.md`: this report, evidence references, proposed correction and limits.
- `evidence.json`: manually transcribed results of the executed offline reads;
  includes the five raw payloads and exact bag/error timestamps.
- `offline_audit.py`: reusable, file-only inspection source; no ROS node,
  publisher, serial access, SSH, environment setup, or file writes. Caps saved
  anomalies at 256, log correlations at 512 and raw samples at 128 bytes;
  a second IMU pass avoids retaining full-rate history. Large declared servo
  counts are only integers, never allocation sizes. Bag-reader chunk memory
  is managed by rosbag and is outside these application-level caps.
- `test_parsing.py`: unexecuted parser test source for good/bad counts, truncation,
  extra bytes, zero count, malformed IMU, overlap and zero/nonmatching cases.

The saved inspector and tests were reviewed statically, not executed, compiled
or imported. Separate inline bag-reading snippets produced the reported data;
no test suite was run. To use the inspector later in an already configured
ROS1 Python environment, run `python3 offline_audit.py /path/to/existing.bag`.
It prints JSON to stdout; it does not create a recording or connect to ROS.
No claim is made that its runtime behavior or tests have been validated.
The evidence JSON was parsed separately, and each saved hex sample was checked
to contain 68 bytes and the recorded count/stamp. This was an artifact
consistency check, not execution of the saved parser or its tests.

Only this new folder was created. Existing tracked files and source logic
were left unchanged. No branch change, commit, push, pull, fetch, build,
dependency update, deployment, flashing, ROS launch, live topic interaction,
hardware command, or bag recording was performed.
