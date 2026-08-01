cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(PythonInterp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  mujoco_ros_control
  spinal
  pluginlib
  roscpp
  std_msgs
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs mujoco_ros_control pluginlib roscpp spinal std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller
  src/control/gimbalrotor_controller.cpp
  src/control/gimbalrotor_admittance_controller.cpp
  src/control/gimbalrotor_perching_admittance_controller.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model
  src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

add_library(gimbalrotor_navigation
  src/gimbalrotor_navigation.cpp
  src/gimbalrotor_perching_navigation.cpp)
target_link_libraries(gimbalrotor_navigation ${catkin_LIBRARIES})

install(DIRECTORY
  config
  plugins
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

add_custom_target(postprocess_beetle_cutting_mujoco_model ALL
  COMMAND ${PYTHON_EXECUTABLE}
    ${PROJECT_SOURCE_DIR}/scripts/mujoco/postprocess_beetle_cutting_model.py
    --model ${PROJECT_SOURCE_DIR}/mujoco/beetle/robot.xml
    --config ${PROJECT_SOURCE_DIR}/config/beetle/MujocoPerchingCutting.yaml
  DEPENDS generate_mujoco_model
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMENT "Post-processing Beetle MuJoCo cutting model")

catkin_install_python(PROGRAMS
  scripts/mujoco/postprocess_beetle_cutting_model.py
  scripts/tasks/mujoco_cutting_experiment.py
  scripts/analysis/calibrate_mujoco_cutting_profiles.py
  scripts/analysis/analyze_mujoco_cutting_comparison.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
