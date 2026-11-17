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
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  tf
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp spinal nav_msgs sensor_msgs geometry_msgs std_srvs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller src/control/gimbalrotor_controller.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

add_library(gimbalrotor_navigation src/gimbalrotor_navigation.cpp)
target_link_libraries(gimbalrotor_navigation ${catkin_LIBRARIES})

option(GIMBALROTOR_BUILD_NMPC "Build the experimental acados v0.5.0 four-tilt controller" OFF)
if(GIMBALROTOR_BUILD_NMPC)
  set(ACADOS_ROOT "$ENV{ACADOS_SOURCE_DIR}" CACHE PATH "Native acados v0.5.0 installation")
  set(NMPC_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/generated/nmpc")
  set(NMPC_MODEL "gimbalrotor_tilt_quad_servo")
  if(NOT EXISTS "${NMPC_GENERATED}/acados_solver_${NMPC_MODEL}.c" OR
     NOT EXISTS "${NMPC_GENERATED}/nmpc_config.h")
    message(FATAL_ERROR "Generate NMPC sources first: python3 scripts/nmpc/gen_nmpc_code.py -m GimbalrotorTiltQuadServoNMPC (see NMPC.md)")
  endif()
  find_path(ACADOS_INCLUDE acados_c/ocp_nlp_interface.h PATHS "${ACADOS_ROOT}/include" NO_DEFAULT_PATH)
  find_library(ACADOS_LIBRARY acados PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH)
  find_library(HPIPM_LIBRARY hpipm PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH)
  find_library(BLASFEO_LIBRARY blasfeo PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH)
  if(NOT ACADOS_INCLUDE OR NOT ACADOS_LIBRARY OR NOT HPIPM_LIBRARY OR NOT BLASFEO_LIBRARY)
    message(FATAL_ERROR "Build/install native acados v0.5.0 first and set ACADOS_SOURCE_DIR; ordinary PID builds need GIMBALROTOR_BUILD_NMPC=OFF")
  endif()
  file(GLOB NMPC_MODEL_SOURCES "${NMPC_GENERATED}/${NMPC_MODEL}_model/*.c")
  file(GLOB NMPC_COST_SOURCES "${NMPC_GENERATED}/${NMPC_MODEL}_cost/*.c")
  add_library(gimbalrotor_nmpc_solver SHARED
    "${NMPC_GENERATED}/acados_solver_${NMPC_MODEL}.c"
    ${NMPC_MODEL_SOURCES} ${NMPC_COST_SOURCES})
  # Override the package's inherited C++ flag for generated C sources only.
  set_property(TARGET gimbalrotor_nmpc_solver PROPERTY COMPILE_OPTIONS "-std=c99")
  target_include_directories(gimbalrotor_nmpc_solver PUBLIC
    "${NMPC_GENERATED}" "${ACADOS_INCLUDE}"
    "${ACADOS_INCLUDE}/blasfeo/include" "${ACADOS_INCLUDE}/hpipm/include")
  target_link_libraries(gimbalrotor_nmpc_solver ${ACADOS_LIBRARY} ${HPIPM_LIBRARY} ${BLASFEO_LIBRARY} m)
  add_library(gimbalrotor_nmpc_controller SHARED src/control/nmpc_controller.cpp src/control/nmpc_solver.cpp)
  target_include_directories(gimbalrotor_nmpc_controller PRIVATE ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(gimbalrotor_nmpc_controller gimbalrotor_nmpc_solver gimbalrotor_robot_model ${catkin_LIBRARIES})
  if(catkin_EXPORTED_TARGETS)
    add_dependencies(gimbalrotor_nmpc_controller ${catkin_EXPORTED_TARGETS})
  endif()
  set_target_properties(gimbalrotor_nmpc_solver gimbalrotor_nmpc_controller PROPERTIES
    INSTALL_RPATH "${ACADOS_ROOT}/lib;\$ORIGIN")
  install(TARGETS gimbalrotor_nmpc_solver gimbalrotor_nmpc_controller gimbalrotor_robot_model
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

install(DIRECTORY scripts/nmpc DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
install(FILES NMPC.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
