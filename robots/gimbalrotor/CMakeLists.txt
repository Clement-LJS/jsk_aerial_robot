cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  geometry_msgs
  nav_msgs
  tf_conversions
  spinal
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    aerial_robot_control
    aerial_robot_estimation
    aerial_robot_model
    aerial_robot_msgs
    geometry_msgs
    nav_msgs
    tf_conversions
    pluginlib
    roscpp
    sensor_msgs
    spinal
    std_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller
  src/control/gimbalrotor_controller.cpp
  src/control/gimbalrotor_admittance_controller.cpp
  src/control/gimbalrotor_perching_admittance_controller.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model
  src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

add_library(roll_yaw_perching_core
  src/roll_yaw_perching_kinematics.cpp
  src/roll_yaw_orientation_qp.cpp
  src/roll_yaw_cutting_trajectory.cpp)
target_link_libraries(roll_yaw_perching_core ${catkin_LIBRARIES})

add_library(gimbalrotor_navigation
  src/gimbalrotor_navigation.cpp
  src/gimbalrotor_perching_navigation.cpp
  src/gimbalrotor_multilink_perching_navigation.cpp
  src/gimbalrotor_pitch_only_perching_navigation.cpp
  src/gimbalrotor_roll_yaw_perching_navigation.cpp)
target_link_libraries(gimbalrotor_navigation roll_yaw_perching_core ${catkin_LIBRARIES})
add_dependencies(gimbalrotor_navigation ${catkin_EXPORTED_TARGETS})

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(roll_yaw_perching_test test/roll_yaw_perching_test.cpp)
  if(TARGET roll_yaw_perching_test)
    target_link_libraries(roll_yaw_perching_test roll_yaw_perching_core ${catkin_LIBRARIES})
  endif()
  add_subdirectory(test)
endif()
