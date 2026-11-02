GIMBALROTOR DEBUGGING TOOLS

Canonical layout
  debug/simulate_run/       Passive ROS/USB recording tools and optional bench launch
  debug/inspection/         Existing investigation scripts, reports and saved evidence
  debug/reproduce_rosserial.py  Source-pinned offline corruption reproduction
  debug/source_manifest.json   Hashes and roles of the reviewed production sources
  debug/SOURCE_PATH.txt      Bringup -> allocation -> FC -> telemetry explanation
  debug/LIVE_VERIFICATION.txt  Separate future firmware verification procedure
  debug/results/            Results of offline verification in this checkout

The old gimbalrotor/simulate_run and gimbalrotor/inspection paths are compatibility
symlinks. Keep them while recordings or old commands may still use those paths.
No external rosbag directory is moved. Historical JSON/metadata keeps its original
paths as provenance; those strings are not rewritten to pretend the recording
was made in the new location. ST-Link dumps are historical evidence, not firmware
to flash. No production controller/firmware/launch file is changed by this work.

Run the offline reproduction from the gimbalrotor directory:

  python3 -B debug/reproduce_rosserial.py

The default reads the three existing 5_pidtuning diagnostic bag splits. It prints
JSON, verifies the reviewed source hashes, demonstrates the possible interleaving,
checks actual PWM/servo bytes and reproduces the recorded Python decoding error.
It imports the existing rosbag file reader; it does not source a ROS setup file,
contact the ROS master, start a node, replay a bag, send commands or access USB.
It does not write results unless you redirect stdout yourself.

To run without any ROS Python import or bag access:

  python3 -B debug/reproduce_rosserial.py --model-only

To read another completed diagnostic folder:

  python3 -B debug/reproduce_rosserial.py --run /absolute/path/to/completed/run

The bag mode expects one run's chronological split files, not unrelated flights.
It retains at most 16 malformed examples and 32 error logs. It never allocates
an array using the bogus servo count. The bounded Python decoder failure differs
from C++ vector allocation; do not use malformed packets to provoke live OOM.

What a successful result means
  interleaved: valid frame checksum, invalid servo count 65537000
  serialized_schedule: valid checksum and the original eight servo entries
  recording: one malformed servo packet in 5_pidtuning, ARM_OFF, exact recorded
             PWM prefix, real decoding exception and recent control-message ages

This demonstrates a possible execution of the reviewed serialization algorithm.
It does not execute the full allocator, RTOS scheduler, STM32 binary or a physical
flight model. The recorded allocation/command samples establish the active path;
SOURCE_PATH.txt explains where that path meets the shared buffer. The protected
case is a schedule comparison, not an installed mutex fix. Source hashes detect
drift and stop the script: review changed sources before refreshing the manifest.

Current capture commands (for a later run; do not start duplicate recorders):
  cd /home/clement/rosbag/year2026/11Nov/261101
  python3 /home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor/debug/simulate_run/bench_capture.py

The current directory determines where the timestamped recording folder is made.
Onboard capture uses the onboard user's existing checkout and ROS environment.
ROS master may remain on the laptop. See simulate_run/README.txt for the passive
recorder. USB capture must run where the physical adapter is connected; the VIM4
5.4.125 kernel currently lacks usbmon, so capture_usb.py cannot capture there.

Offline tests (no workspace build, nodes or devices):
  python3 -B -m unittest discover -s debug -p test_reproduction.py
  python3 -B -m unittest discover -s debug/simulate_run -p test_offline.py
  python3 -B -m unittest discover -s debug/inspection/rosserial_20261101 -p test_audit.py

Live verification remains a later, explicitly scheduled operation after the
current flight/recording ends. See LIVE_VERIFICATION.txt for the evidence required
before calling the deployed cause proven or the firmware fix verified.
