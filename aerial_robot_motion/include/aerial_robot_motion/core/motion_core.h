#pragma once
#include <aerial_robot_motion/core/model_adapter.h>
#include <aerial_robot_motion/cost/base_plugin.h>
#include <aerial_robot_motion/constraint/base_plugin.h>
#include <aerial_robot_motion/command/motion_command_manager.h>
#include <aerial_robot_motion/execution/whole_body_reference_bridge.h>
#include <aerial_robot_motion/solver/qpoases_solver.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
namespace aerial_robot_motion
{
class MotionCore
{
public:
  MotionCore(ros::NodeHandle nh, ros::NodeHandle pnh);
private:
  void update(const ros::TimerEvent&);
  void odometry(const nav_msgs::OdometryConstPtr&);
  void cogOdometry(const nav_msgs::OdometryConstPtr&);
  void jointState(const sensor_msgs::JointStateConstPtr&);
  bool perching(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
  bool resetTarget(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  bool measurements(MotionState&, std::string&);
  void resetOptimizationState();
  void diagnostic(bool valid, const std::string& text, const MotionContext* context = nullptr,
                  const QPProblem* problem = nullptr, const Eigen::VectorXd* solution = nullptr, double seconds = 0);
  ros::NodeHandle nh_, pnh_;
  ModelAdapter model_;
  pluginlib::ClassLoader<cost::Base> cost_loader_;
  pluginlib::ClassLoader<constraint::Base> constraint_loader_;
  std::vector<boost::shared_ptr<cost::Base>> costs_;
  std::vector<boost::shared_ptr<constraint::Base>> constraints_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener tf_listener_;
  command::MotionCommandManager commands_;
  WholeBodyReferenceBridge bridge_;
  QpOasesSolver solver_;
  ros::Subscriber odom_sub_, cog_sub_, joint_sub_, flight_sub_, inhibit_sub_;
  ros::Publisher diagnostics_;
  ros::ServiceServer perching_service_, reset_service_;
  ros::Timer timer_;
  nav_msgs::Odometry odom_, cog_;
  sensor_msgs::JointState joints_;
  ros::Time last_solved_stamp_, flight_stamp_;
  int flight_state_ = -1, hover_state_ = 5;
  bool have_odom_ = false, have_cog_ = false, have_joints_ = false;
  bool publish_commands_ = false, inhibited_ = false, perched_ = false, has_contact_plugin_ = false;
  Eigen::Vector3d hinge_axis_ = Eigen::Vector3d::UnitY(), hinge_local_ = Eigen::Vector3d::UnitY();
  Eigen::Vector3d locked_position_ = Eigen::Vector3d::Zero(), locked_axis_ = Eigen::Vector3d::UnitY();
  std::string world_, hinge_frame_;
  double rate_ = 50, timeout_ = 0.2, max_dt_ = 0.1, max_skew_ = 0.05;
  double regularization_ = 1e-6, contact_tolerance_ = 0.005, contact_angle_tolerance_ = 0.02;
};
}
