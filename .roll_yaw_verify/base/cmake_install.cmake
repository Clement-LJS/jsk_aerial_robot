# Install script for directory: /home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_base

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install" TYPE PROGRAM FILES "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install" TYPE PROGRAM FILES "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/setup.bash;/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install" TYPE FILE FILES
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/setup.bash"
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/setup.sh;/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install" TYPE FILE FILES
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/setup.sh"
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/setup.zsh;/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install" TYPE FILE FILES
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/setup.zsh"
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install" TYPE FILE FILES "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/aerial_robot_base.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aerial_robot_base/cmake" TYPE FILE FILES
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/aerial_robot_baseConfig.cmake"
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/catkin_generated/installspace/aerial_robot_baseConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aerial_robot_base" TYPE FILE FILES "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_base/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aerial_robot_base" TYPE DIRECTORY FILES "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_base/include/aerial_robot_base/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaerial_robot_base.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaerial_robot_base.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaerial_robot_base.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/devel/lib/libaerial_robot_base.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaerial_robot_base.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaerial_robot_base.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaerial_robot_base.so"
         OLD_RPATH "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/devel/lib:/home/clement/ros/jsk_aerial_robot_ws/devel/.private/aerial_robot_estimation/lib:/opt/ros/one/lib:/home/clement/ros/jsk_aerial_robot_ws/devel/.private/aerial_robot_model/lib:/home/clement/ros/jsk_aerial_robot_ws/devel/.private/spinal/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaerial_robot_base.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aerial_robot_base/aerial_robot_base_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aerial_robot_base/aerial_robot_base_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aerial_robot_base/aerial_robot_base_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/aerial_robot_base" TYPE EXECUTABLE FILES "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/devel/lib/aerial_robot_base/aerial_robot_base_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aerial_robot_base/aerial_robot_base_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aerial_robot_base/aerial_robot_base_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aerial_robot_base/aerial_robot_base_node"
         OLD_RPATH "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/devel/lib:/home/clement/ros/jsk_aerial_robot_ws/devel/.private/aerial_robot_estimation/lib:/opt/ros/one/lib:/home/clement/ros/jsk_aerial_robot_ws/devel/.private/aerial_robot_model/lib:/home/clement/ros/jsk_aerial_robot_ws/devel/.private/spinal/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aerial_robot_base/aerial_robot_base_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/aerial_robot_base" TYPE DIRECTORY FILES "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_base/bin" USE_SOURCE_PERMISSIONS)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aerial_robot_base" TYPE DIRECTORY FILES
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_base/launch"
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_base/scripts"
    USE_SOURCE_PERMISSIONS)
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
