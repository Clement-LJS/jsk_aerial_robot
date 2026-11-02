# Inspection of the two 2026-10-30 14:30 recordings

No recurrence of the previously observed malformed servo-array payloads,
std::bad_alloc, or Buffer Overrun was found. There is a separate, substantial
reception-timing irregularity: pauses followed by batches of messages.
Existing source code was not changed. No ROS nodes, playback, SSH, or hardware
commands were used for this analysis.

## Coverage

Inputs are in `/home/clement/rosbag/year2026/10Oct/261030/`.
Times below are JST on 2026-10-30.

| Check | `1.bag` | `rosserial_diag_2026-10-30-14-30-32.bag` |
| --- | ---: | ---: |
| Bag time interval | 14:30:31.175–14:32:11.932 | 14:30:32.665–14:32:13.293 |
| IMU messages | 18,095 | 18,065 |
| Servo messages | 5,023 | 5,018 |
| Malformed servo count/length | 0 | 0 |
| Matching deserialization errors | 0 | 0 |
| Mean IMU rate over received topic span | 179.99 Hz | 179.99 Hz |
| Mean servo rate over received topic span | 49.98 Hz | 49.99 Hz |
| IMU reception gaps greater than 100 ms | 30 | 28 |
| Servo reception gaps greater than 100 ms | 31 | 33 |
| Maximum IMU reception gap | 295.63 ms | 293.26 ms |
| Maximum servo reception gap | 279.65 ms | 279.68 ms |

These are overlapping recordings of the same activity, not independent trials.
Within their common embedded-timestamp interval, trimming 20 ms at either
boundary, all 17,808 IMU payloads and 4,947 servo payloads match byte-for-byte
with matching multiplicity. Neither recorder has an extra/missing payload
relative to the other inside this comparison interval. This does not rule out
loss before the stream reached both recorders.

## Payload integrity and telemetry

- Every IMU payload is 60 bytes, with finite float values and valid timestamp
  nanoseconds. Quaternion norms range from approximately 0.999999939 to
  1.000000068. No embedded timestamps go backward or repeat.
- Every servo payload is 68 bytes, declares eight records, and contains indices
  0 through 7 once each. All recorded servo error fields are zero. Reported
  temperatures are 22–33 degrees in the message's temperature field.
- Every diagnostic-bag servo payload was also searched for an exact IMU prefix
  of at least 16 bytes with at least four distinct byte values, at any payload
  offset, against IMU records within +/-1 second of bag reception time. There
  were no matches. This does not detect every conceivable byte corruption.
- Acceleration magnitude ranges from 8.37 to 10.76 m/s² and angular-rate
  magnitude reaches 0.851 rad/s. These finite excursions are not independently
  evidence of corruption. This is not a complete physical sensor validation.
- Magnetometer values were checked for finiteness. Their physical unit scaling
  was not certified; the `.msg` unit comment alone does not verify the flashed
  sensor conversion. No magnetometer physical-range assertion is made.

Only IMU/servo telemetry and ROS logs were analyzed. Actuation, flight-control,
navigation, and unrelated topics in the larger bag were not evaluated.

## Reception stalls followed by bursts

At approximately **14:30:54.696**, the diagnostic recorder has a **293.26 ms
IMU reception gap**, but the embedded timestamps of the messages immediately
before and after that gap are only **6 ms** apart. In the following **20 ms**,
the recorder receives **56 IMU messages spanning 305 ms of embedded time**.
Servo data simultaneously has a **279.68 ms reception gap**, followed by
**15 messages spanning 280 ms of embedded time** within 20 ms.

The larger bag shows the same event: 295.63 ms followed by 57 IMU records
spanning 311 ms within 20 ms. Other prominent paired pauses occur near
14:31:00.736 and 14:32:03.89. The plot in `new_bags_1430_timing.png` shows the
diagnostic recorder's reception intervals against embedded timestamp steps.

This strongly supports a queued/backlogged stream or delayed callback
execution followed by burst reception, rather than an equivalent pause in
IMU sample production. It does not identify the responsible layer. Candidates
include transport/network buffering, bridge scheduling, or recorder/host
scheduling. Both recording processes could be affected by a shared path or
host. Bags do not record monotonic receive time, so clock adjustments cannot
be excluded purely from recorder timestamps. The accompanying burst counts
and source spans make simple average-rate statistics insufficient.

Do not interpret these gaps as measured one-way latency or evidence that the
onboard control subscriber experienced the same pauses. There is no verified
clock synchronization or onboard-versus-laptop capture comparison here.

## Embedded-clock changes and logs

Each topic exhibits 40 changes in its sub-millisecond timestamp component
over about 100 seconds, with matching changes across IMU and servo data.
Embedded IMU intervals span 0.084–14.070 ms; servo intervals span
13.084–27.448 ms. This is consistent with shared rosserial time-offset
adjustments, not evidence that every interval variation is a sampling fault.
The inspected generated `ros/node_handle.h:334` requests synchronization on a
`SYNC_SECONDS * 500` millisecond interval; `SYNC_SECONDS` is 5 (line 73), and
`syncTime`/`setNow` adjust the shared offset (lines 361–388). This is supporting
source evidence, not proof of the exact firmware flashed during recording.

There is one unique WARN entry, at **14:30:51.093**, from
`/gimbalrotor/aerial_robot_base_node`: `start roll/pitch I control`.
It appears in both bags and additionally in `/rosout_agg` in `1.bag`.
It reports a controller transition; it is not a deserialization exception.
No controller logic was inspected or changed. Other recorded log entries are
INFO; no ERROR/FATAL entries or recorder buffer-overflow warnings were found.

## Interpretation and next evidence

The previous corruption did not recur in the observed payloads. This short
recording does not eliminate an intermittent serialization race. The new
finding is delivery/recording jitter, which must be kept separate from that
byte-corruption hypothesis.

A useful next observation is the same telemetry-only recording simultaneously
on the onboard PC and laptop, while keeping existing robot behavior unchanged.
An onboard stream that is smooth while the laptop stream batches would narrow
the issue to the downstream network/laptop path. Batching in both would keep
the common upstream path and onboard scheduling under consideration. This is
a proposed diagnostic comparison, not a command executed in this inspection.

## Artifacts and method

- `new_bags_1430.json`: numeric checks, selected-message counts, timing
  distributions, largest gaps, burst evidence, payload comparisons and log
  observations. Raw message bytes were inspected without allocating arrays
  from their embedded counts.
- `new_bags_1430_timing.png`: reception-versus-source interval plot.
- `new_bags_1430_report.md`: this report.

These results came from offline inline Python readers using the installed
rosbag library, plus plotting. The previously saved `offline_audit.py` and
`test_parsing.py` were not run. No production source files were edited, and
no builds, tests, commits, pushes, or live robot operations were performed.
