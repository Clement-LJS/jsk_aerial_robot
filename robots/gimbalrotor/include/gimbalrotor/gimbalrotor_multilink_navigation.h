// -*- mode: c++ -*-

#pragma once

#include <ros/ros.h>
#include <aerial_robot_control/flight_navigation.h>

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <aerial_robot_msgs/FlightNav.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <spinal/DesireCoord.h>

#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/frames.hpp>

namespace aerial_robot_navigation
{

class GimbalrotorMultilinkNavigator : public BaseNavigator
{
public:
  GimbalrotorMultilinkNavigator();
  virtual ~GimbalrotorMultilinkNavigator() = default;

  virtual void initialize(ros::NodeHandle nh,
                          ros::NodeHandle nhp,
                          boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                          double loop_du) override;

  virtual void update() override;
  virtual void reset() override;
  virtual void halt();

protected:
  virtual void rosParamInit() override;
  virtual void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;

  void targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg);
  void targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg);

  void baselinkRotationProcess();
  void pitchLinkCompensationProcess();
  void landingProcess();

  void publishPitchJointCommand(double pitch_joint_cmd);
  double getCurrentPitchJointPosition();

  ros::Publisher target_baselink_rpy_pub_;
  ros::Publisher joint_control_pub_;

  ros::Subscriber final_target_baselink_rot_sub_;
  ros::Subscriber final_target_baselink_rpy_sub_;

  tf::Quaternion curr_target_baselink_rot_;
  tf::Quaternion final_target_baselink_rot_;

  double prev_rotation_stamp_;
  double prev_joint_stamp_;

  double baselink_rot_change_thresh_;
  double baselink_rot_pub_interval_;

  double joint_cmd_pub_interval_;
  double pitch_joint_compensation_sign_;
  double pitch_joint_offset_;
  double pitch_joint_limit_;
  double takeoff_pitch_joint_angle_;
  double landing_pitch_joint_angle_;
  double pitch_joint_land_thresh_;

  std::string pitch_joint_name_;

  bool eq_cog_world_;
  bool keep_hand_horizontal_;
  bool level_flag_;
  bool landing_or_halt_mode_;
};

} // namespace aerial_robot_navigation
