cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  spinal
  pluginlib
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf_conversions
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs geometry_msgs pluginlib roscpp sensor_msgs spinal std_msgs tf_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(gimbalrotor_navigation
  src/gimbalrotor_navigation.cpp
  src/gimbalrotor_perching_navigation.cpp)
target_link_libraries(gimbalrotor_navigation ${catkin_LIBRARIES})

add_library(gimbalrotor_controller
  src/control/gimbalrotor_controller.cpp
  src/control/gimbalrotor_interaction_controller.cpp)
target_link_libraries(
  gimbalrotor_controller
  gimbalrotor_navigation
  ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model
  src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
