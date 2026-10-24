execute_process(COMMAND "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
