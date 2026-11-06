#!/usr/bin/env bash
# Run from an existing ROS/dependency environment. No master, nodes or hardware.
set -euo pipefail
package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
check_dir=$(mktemp -d /tmp/gimbalrotor-checks.XXXXXX)
export ROS_HOME="$check_dir/ros"
export PYTHONDONTWRITEBYTECODE=1
printf 'Build and check artifacts: %s\n' "$check_dir"
cmake -S "$package_dir" -B "$check_dir/build" \
  -DCATKIN_DEVEL_PREFIX="$check_dir/devel" \
  -DCMAKE_INSTALL_PREFIX="$check_dir/install" \
  -DCATKIN_ENABLE_TESTING=ON -DCMAKE_BUILD_TYPE=Release > "$check_dir/configure.log" 2>&1 || {
  tail -60 "$check_dir/configure.log"; exit 1;
}
cmake --build "$check_dir/build" -j2 > "$check_dir/build.log" 2>&1 || {
  tail -60 "$check_dir/build.log"; exit 1;
}
cmake --build "$check_dir/build" --target roll_yaw_core_test -j2 >> "$check_dir/build.log" 2>&1 || {
  tail -60 "$check_dir/build.log"; exit 1;
}
xacro "$package_dir/robots/beetle/gimbalrotor.urdf.xacro" \
  use_roll_yaw_joint:=true use_pitch_joint_only:=false use_roll_joint:=false \
  use_yaw_joint:=false use_hand:=false > "$check_dir/robot.urdf"
export LD_LIBRARY_PATH="$check_dir/devel/lib:${LD_LIBRARY_PATH:-}"
ROLL_YAW_TEST_URDF="$check_dir/robot.urdf" \
  "$check_dir/devel/lib/gimbalrotor/roll_yaw_core_test" \
  --gtest_output="xml:$check_dir/tests.xml" | tee "$check_dir/core-tests.log"
python3 "$package_dir/test/test_roll_yaw_interfaces.py" 2>&1 | tee "$check_dir/interface-tests.log"
