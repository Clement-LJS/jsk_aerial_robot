# CMake generated Testfile for 
# Source directory: /home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor
# Build directory: /home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/gimbalrotor
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_gimbalrotor_gtest_roll_yaw_perching_test "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/gimbalrotor/catkin_generated/env_cached.sh" "/usr/bin/python3" "/usr/share/catkin/cmake/test/run_tests.py" "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/gimbalrotor/test_results/gimbalrotor/gtest-roll_yaw_perching_test.xml" "--return-code" "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/devel/lib/gimbalrotor/roll_yaw_perching_test --gtest_output=xml:/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/gimbalrotor/test_results/gimbalrotor/gtest-roll_yaw_perching_test.xml")
set_tests_properties(_ctest_gimbalrotor_gtest_roll_yaw_perching_test PROPERTIES  _BACKTRACE_TRIPLES "/usr/share/catkin/cmake/test/tests.cmake;160;add_test;/usr/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/usr/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor/CMakeLists.txt;79;catkin_add_gtest;/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor/CMakeLists.txt;0;")
subdirs("test")
