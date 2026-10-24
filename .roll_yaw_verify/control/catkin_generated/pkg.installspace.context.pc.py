# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "aerial_robot_estimation;aerial_robot_model;aerial_robot_msgs;dynamic_reconfigure;pluginlib;roscpp;spinal;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lcontrol_utils;-lflight_control_pluginlib;-lflight_navigation;-ltrajectory_generation".split(';') if "-lcontrol_utils;-lflight_control_pluginlib;-lflight_navigation;-ltrajectory_generation" != "" else []
PROJECT_NAME = "aerial_robot_control"
PROJECT_SPACE_DIR = "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install"
PROJECT_VERSION = "1.3.6"
PROJECT_LIB_DIR = "lib"
