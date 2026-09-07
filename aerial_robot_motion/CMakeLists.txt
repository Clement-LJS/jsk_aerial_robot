cmake_minimum_required(VERSION 3.10)
project(aerial_robot_motion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(catkin REQUIRED COMPONENTS
  aerial_robot_model aerial_robot_msgs diagnostic_msgs geometry_msgs nav_msgs
  pluginlib roscpp rospy sensor_msgs std_msgs std_srvs tf tf2_ros
  tf2_geometry_msgs kdl_parser)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

include(cmake/qpOASES.cmake)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aerial_robot_motion
  CATKIN_DEPENDS aerial_robot_model aerial_robot_msgs diagnostic_msgs
    geometry_msgs nav_msgs pluginlib roscpp sensor_msgs std_msgs std_srvs
    tf tf2_ros tf2_geometry_msgs kdl_parser
  DEPENDS EIGEN3 orocos_kdl)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
add_library(aerial_robot_motion
  src/core/model_adapter.cpp src/core/motion_core.cpp
  src/cost/cartesian_pose.cpp src/cost/state_velocity.cpp src/cost/nominal_posture.cpp
  src/constraint/state_limit.cpp src/constraint/joint_limit.cpp src/constraint/revolute_contact.cpp
  src/solver/qpoases_solver.cpp src/command/motion_command_manager.cpp
  src/compliance/cartesian_admittance.cpp src/execution/whole_body_reference_bridge.cpp)
target_include_directories(aerial_robot_motion PRIVATE ${QPOASES_INCLUDE_DIR})
target_link_libraries(aerial_robot_motion ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} motion_qpoases)
add_dependencies(aerial_robot_motion ${catkin_EXPORTED_TARGETS})

add_executable(aerial_robot_motion_node src/core/node.cpp)
target_link_libraries(aerial_robot_motion_node aerial_robot_motion ${catkin_LIBRARIES})
catkin_install_python(PROGRAMS scripts/keyboard_motion.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS aerial_robot_motion aerial_robot_motion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(test_name test_qp test_command test_admittance)
    catkin_add_gtest(${test_name} test/${test_name}.cpp test/main.cpp)
    if(TARGET ${test_name})
      target_link_libraries(${test_name} aerial_robot_motion ${catkin_LIBRARIES})
    endif()
  endforeach()
  add_rostest_gtest(model_kinematics test/model.test test/model_kinematics.cpp test/main.cpp)
  target_link_libraries(model_kinematics aerial_robot_motion ${catkin_LIBRARIES})
  add_rostest(test/runtime.test DEPENDENCIES aerial_robot_motion_node)
endif()
