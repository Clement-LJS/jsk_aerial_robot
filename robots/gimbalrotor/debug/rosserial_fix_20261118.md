# STM32 rosserial packet corruption and servo feedback repair

## Change record

| Field | Details |
| --- | --- |
| Implementation and documentation date | 2026-11-18, Asia/Tokyo (JST) |
| Author | OpenAI Codex, AI coding assistant, implementing the user's requested changes |
| Primary hardware | STM32H7_v2, identified by the user as Spinal 4.1 |
| Repositories | `jsk_aerial_robot` and the separate `rosserial` repository |
| JSK branch used for the repair | `develop/pochitab_passiveperching/main` |
| JSK baseline before implementation | `60fe15c073cca53fc931d70212d02c8635934725` |
| JSK HEAD observed before documentation cleanup | `83307fbc` — `[Spinal] fix(rosserial): prevent concurrent packet corruption and reject malformed servo feedback` |
| rosserial HEAD observed before cleanup | `7cd68bd`; receiver changes were still local and uncommitted |
| Deployment performed by this work | Local source edits and builds only; no firmware flashing or robot-node restart |

The assistant did not commit or push changes. The JSK commit listed above already existed when this cleanup began. This document describes the functional repair and the subsequent user-requested removal of its test/support files.

## Problem and recorded evidence

The investigation used five flights and their corresponding inspection recordings under:

```text
/home/clement/rosbag/year2026/11Nov/261117/
```

The most recent flight was `5cutattempted_autoscriptfailed_manualcut.bag`. During that flight, the servo bridge reported an input-voltage error for gimbal 1, alongside other servo error bits. A separate occurrence produced a `Buffer Overrun` during ROS message deserialization.

The raw recordings showed that these were malformed `spinal/ServoStates` payloads containing prefixes copied from other topic payloads:

- Across all five flights, 12 unique malformed servo payloads were identified: 10 matched PID-message prefixes and two matched PWM-message prefixes. Copies recorded in both the main and inspection bags are not additional independent incidents.
- At approximately **11:37:55 JST** in flight 5, a 68-byte servo payload contained a 26-byte PID prefix. Its timestamp/count were malformed. Decoding it produced a bogus gimbal-1 record with error byte `255`, including the input-voltage bit, and an incorrect angle. This affected feedback, not just the displayed warning.
- At approximately **11:42:01 JST**, another 68-byte servo payload contained a 60-byte PID prefix. The corrupted array count was 23, which would require 173 bytes (`12 + 7 × 23`), explaining the downstream deserialization overrun.
- All structurally valid servo samples in the inspection recordings had zero servo error bits. This supports corruption as the explanation for the observed warning, but does not establish that an independent electrical fault could never occur.
- The separately quoted timestamp `1794883627.787016709` corresponds to **11:47:07.787 JST**, after flight 5's bag ended at about 11:42:48. A corresponding error was found in the ROS log, but its raw payload was not captured in these bags.

## Root cause supported by the code and recordings

The STM32 core and servo tasks share a ROS node handle. The original rosserial `publish(id, msg)` serialized messages into a shared `message_out` buffer, then added the frame header and checksum.

The existing mutex in `STM32Hardware::write()` protected the transmit queue only. It was acquired **after serialization**, leaving the shared packet buffer exposed to concurrent publishers. One task could overwrite another task's payload before its frame was copied to the queue. A checksum computed after such mixing can still be valid, so the wire checksum alone cannot reject every mixed packet.

The matching PID/PWM prefixes and this unprotected shared-buffer path strongly support a publication race. No direct on-board scheduling trace was captured; the diagnosis should not be described as a measured trace of the precise interrupt/task interleaving.

On the host, rosserial forwarded the payload using `ShapeShifter` without checking the servo schema. Consequently, malformed data could either reach a typed subscriber and cause an overrun, or decode into misleading servo values. The servo bridge also updated records as it iterated, allowing a malformed later record to leave earlier updates applied.

## Functional changes

### 1. Serialize outgoing packets under a dedicated firmware mutex

`NodeHandleStm_` now overrides `publish(int id, const Msg* msg)`. With RTOS support enabled, it acquires a dedicated recursive CMSIS mutex before calling the parent publisher and releases it after serialization, framing/checksum creation, and the transport write/copy have finished.

All virtual publication calls, including ordinary publishers, negotiation messages and ROS logging, use this path. The parameterless `publish()` overload that drains the UART transmit queue remains outside the new lock; waiting for DMA completion does not hold the serialization mutex.

The new mutex is separate from the existing transmit-queue mutex. It is recursive because the parent publisher's error path can publish a log message recursively. Reusing the ordinary transmit-queue mutex would risk a nested-lock deadlock.

The mutex is initialized after transport setup for both UART and Ethernet initialization paths. Calls before initialization, failed mutex creation/acquisition, and interrupt-context calls return `-1` without publishing. These failure paths do not attempt ROS logging, which would re-enter publication. The non-RTOS path continues to call the parent publisher directly.

Recursive mutex support is enabled in a CubeMX-preserved user section of `FreeRTOSConfig.h` for STM32H7_v2. STM32F7 and STM32H7 receive the same configuration addition because all three boards use the shared wrapper. Only STM32H7_v2 was fully cross-built during this work.

### 2. Reject malformed servo payloads before host forwarding

The rosserial server now recognizes the `spinal/ServoStates` datatype and checks its known MD5/schema before forwarding it. The supported MD5 is:

```text
8ab23ee17ec1578fba9ec2244276b0ef
```

The allocation-free validator checks:

- At least 12 header bytes are present: seconds, nanoseconds and array count.
- The remaining bytes are an exact multiple of the seven-byte servo record size.
- The advertised array count equals the count implied by the actual byte length.
- Timestamp nanoseconds are below `1,000,000,000`.
- There are at most 256 records and no duplicate uint8 servo indices.

It derives the record count from the available bytes rather than multiplying or allocating from an untrusted count. Rejected packets produce a throttled warning and are not forwarded to ROS subscribers. An unsupported ServoStates MD5 is also rejected rather than parsed using the wrong layout.

The generic validator does not require exactly eight servos or interpret physical angle/load plausibility. It preserves real hardware error bits, including `255`; structural corruption is what triggers rejection. A future intentional message-schema change requires updating this validator and its MD5 check together.

### 3. Validate a whole servo sample before updating feedback

`ServoBridge::servoStatesCallback()` now validates timestamp nanoseconds, record count and duplicate IDs before changing feedback, filter state or stored error history. For an independent servo group, it also validates the group and every supplied servo ID before applying any record.

Common packets can legitimately contain channels without configured joint handlers—for example channels 4/5 on this beetle configuration. Those channels remain ignored, preserving existing behavior. The bridge does not impose a new requirement that every common packet contain every configured servo.

This bridge check also applies to alternate ROS publishers that bypass rosserial. It supplements the raw receiver check; it cannot replace validation before typed deserialization/allocation.

### 4. Restore servo-error-cleared reporting

`reportServoErrorState()` now saves whether the previous error was nonzero before updating the error-state map. Previously, the map entry was overwritten before checking the previous value, suppressing the expected `error cleared` log. Real error reporting remains enabled.

## Production files changed

Paths below are relative to the explicitly named repository root. Combining a root and its relative path gives the full local path. Both repositories are required to carry the complete repair.

### Repository: jsk_aerial_robot

Root:

```text
/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/
```

| Relative path | Change |
| --- | --- |
| `aerial_robot_nerve/spinal/src/ros_lib/ros/node_handle_stm.h` | Persistent shared firmware publication-lock implementation and initialization/failure handling. |
| `aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/Inc/FreeRTOSConfig.h` | Enable recursive mutex support for the target board. |
| `aerial_robot_nerve/spinal/mcu_project/boards/stm32H7/Inc/FreeRTOSConfig.h` | Keep the other H7 board compatible with the shared wrapper. |
| `aerial_robot_nerve/spinal/mcu_project/boards/stm32F7/Inc/FreeRTOSConfig.h` | Keep the F7 board compatible with the shared wrapper. |
| `aerial_robot_model/src/servo_bridge/servo_bridge.cpp` | Validate whole feedback samples and restore error-cleared logging. |

### Repository: rosserial

Root:

```text
/home/clement/ros/jsk_aerial_robot_ws/src/rosserial/
```

| Relative path | Change |
| --- | --- |
| `rosserial_server/include/rosserial_server/topic_handlers.h` | Modified: run ServoStates schema/payload checks before ShapeShifter forwarding. |
| `rosserial_server/include/rosserial_server/servo_states_validation.h` | Added: allocation-free raw ServoStates validator. This header is functional code and must be retained. |

### Generated firmware headers refreshed

The following local generated copies were refreshed from the persistent wrapper above. They are normally ignored by Git and should not be treated as the sole source of the fix:

```text
/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_nerve/spinal/mcu_project/boards/stm32F7/ros_lib/ros/node_handle_stm.h
/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_nerve/spinal/mcu_project/boards/stm32H7/ros_lib/ros/node_handle_stm.h
/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/ros_lib/ros/node_handle_stm.h
```

The existing `spinal/scripts/make_libraries.py` generator copies the persistent wrapper into generated `ros_lib` directories. It was not modified. Keeping the persistent source and the RTOS configuration changes is necessary for future regeneration/builds.

## Validation completed before removing the tests

These are historical results from the implementation session on 2026-11-18. The test sources and runners were subsequently removed at the user's request; this document does not imply that those runners remain available.

| Check | Result |
| --- | --- |
| Full clean STM32H7_v2 compile and link, using the actual Cortex-M7 toolchain and board RTOS configuration in a temporary source copy | Passed; linked map contained the new publication override and recursive mutex functions. Existing unrelated compiler warnings remained. |
| ROS `servo_bridge_node` and rosserial serial/socket/UDP receiver builds | Passed; local workspace binaries rebuilt. |
| Host concurrency exercise using the real rosserial parent/wrapper with mock CMSIS and transport | 20,000 concurrent frames remained intact. Virtual dispatch, recursion, initialization, acquisition failure, interrupt rejection, Ethernet initialization and unlocked UART queue drain were checked. |
| Non-RTOS compatibility exercise | Passed with sequential publication. |
| AddressSanitizer and UndefinedBehaviorSanitizer checks | Passed. LeakSanitizer was disabled because it could not run under the sandbox's ptrace environment. |
| Raw validator boundary checks and recorded corrupt samples | Passed; all 12 unique recorded corrupt payloads rejected; structurally valid real error bits accepted. |
| Raw scan of ten inspection bag segments and five primary flight bags | 234,979 servo packet copies checked; 23 rejected copies represented the 12 unique corrupt payloads; the other 234,956 copies were accepted. Backup/original bags were excluded. |
| Isolated ROS receiver/bridge integration exercise | Passed: rejected samples left feedback/error history unchanged, common extra channels remained supported, error-cleared logging worked, malformed packets were suppressed, and valid packets with real errors were forwarded. |

The tests measured host behavior, not STM32 control-loop scheduling or hardware timing. No flight or stationary hardware capture was performed with the repaired firmware.

## Cleanup requested by the user

The following eight files were removed from `robots/gimbalrotor/debug/rosserial_regression/` because they were test/support material rather than the functional implementation:

```text
fakes/STM32Hardware.h
publication_test.cpp
receiver_bridge_test.cpp
run.py
run_ros.py
validation_test.cpp
recorded_corrupt_servo_payloads.json
CHANGE_REPORT.txt
```

The `fakes/STM32Hardware.h` file was a host test substitute, not the production STM32 hardware driver. The removed text report is replaced by this Markdown document. None of these removed files was referenced by the production build configuration checked during cleanup. The seven production source/configuration files and refreshed generated firmware headers were retained without further functional edits.

Existing workspace build products, earlier bag-analysis artifacts and temporary validation/build records were left in place. They are generated outputs, not additional production source changes, and should not be included in source-code commits.

## Build and deployment handoff

The host binaries were rebuilt under:

```text
/home/clement/ros/jsk_aerial_robot_ws/devel/.private/aerial_robot_model/lib/
/home/clement/ros/jsk_aerial_robot_ws/devel/.private/rosserial_server/lib/rosserial_server/
```

For this already-configured local workspace, the affected host targets can be rebuilt with:

```bash
cmake --build /home/clement/ros/jsk_aerial_robot_ws/build/aerial_robot_model --target servo_bridge_node -j2
cmake --build /home/clement/ros/jsk_aerial_robot_ws/build/rosserial_server --target rosserial_server_serial_node rosserial_server_socket_node rosserial_server_udp_socket_node -j2
```

The firmware validation build was made in a temporary copy, not the original CubeIDE `Debug` directory:

```text
/tmp/gimbalrotor_rosserial_fix/mcu_project/boards/stm32H7_v2/STM32CubeIDE/Debug/spinal.bin
/tmp/gimbalrotor_rosserial_fix/mcu_project/boards/stm32H7_v2/STM32CubeIDE/Debug/spinal.elf
```

The validation binary's SHA-256 was:

```text
dfc49612375138a70338017e08d3ec1f834efc3abc23c1efebe02be1dfa0ae94
```

Build and validation logs were saved under `/tmp/gimbalrotor_rosserial_fix/` as `firmware_build.log`, `bridge_build.log`, `server_build.log`, `regression.log`, `host_tests.log` and `integration.log`. Temporary files may disappear on reboot; this document contains the essential findings independently of them.

To deploy later, regenerate/update the target board's `ros_lib` from the persistent sources as needed, build the **STM32H7_v2** project, flash through the normal board workflow, and restart the relevant host nodes to load the rebuilt binaries. The assistant did not perform flashing or restart robot processes. A source edit or host rebuild alone does not update firmware already running on the board.

After deployment, verify stationary UART/ROS operation and control-loop timing, then capture and inspect a fresh recording using the existing `robots/gimbalrotor/debug/simulate_run/bench_capture.py` workflow before attempting flight. Confirm that malformed ServoStates and cross-topic payload mixing no longer recur. Genuine servo hardware errors must remain visible.

## Scope and remaining limitations

- The repair addresses the demonstrated mixed-payload corruption path and adds host-side protection against structurally malformed servo feedback. The validator cannot detect every possible corruption that happens to preserve a valid schema and plausible fields.
- It does not establish whether an independent physical servo power/voltage problem exists. Real error bits are not suppressed.
- Admittance/tare-loss behavior, observer rejection thresholds and cutting-script timestamp-freshness checks were not modified. Their resolution cannot be claimed from these transport tests.
- Physical scheduling impact, UART operation and flight behavior after flashing remain unverified. Only STM32H7_v2 received a full firmware build in this session.
- Firmware and host receiver changes live in separate Git repositories. Managing only the JSK repository would omit the raw rosserial receiver protection.
