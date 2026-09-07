#include <aerial_robot_motion/execution/whole_body_reference_bridge.h>
#include <aerial_robot_motion/core/ros_conversions.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
namespace aerial_robot_motion
{
void WholeBodyReferenceBridge::initialize(ros::NodeHandle nh, ros::NodeHandle pnh,
                                         const ModelInfo& info, const std::string& world)
{
  info_ = info; world_ = world;
  ros::NodeHandle config(pnh, "bridge");
  config.param("full_attitude", full_attitude_, false);
  std::string nav_topic, joint_topic;
  config.param<std::string>("navigation_topic", nav_topic, "uav/nav");
  config.param<std::string>("joint_command_topic", joint_topic, "joints_ctrl");
  body_ = pnh.advertise<nav_msgs::Odometry>("reference/body", 1);
  joints_ = pnh.advertise<sensor_msgs::JointState>("reference/joints", 1);
  navigation_ = nh.advertise<aerial_robot_msgs::FlightNav>(nav_topic, 1);
  if (!info.joint_names.empty()) joint_command_ = nh.advertise<sensor_msgs::JointState>(joint_topic, 1);
}
void WholeBodyReferenceBridge::publish(const WholeBodyReference& ref, bool execute)
{
  if (!ref.valid || ref.velocity.size() != info_.dimension() || !ref.velocity.allFinite() ||
      !ref.state.root.matrix().allFinite() || !ref.state.cog.matrix().allFinite() ||
      ref.state.joints.size() != static_cast<int>(info_.joint_names.size()) || !ref.state.joints.allFinite())
    throw std::invalid_argument("bridge refused invalid whole-body reference");
  nav_msgs::Odometry body;
  body.header.stamp = ref.state.stamp; body.header.frame_id = world_;
  body.child_frame_id = info_.root_link;
  body.pose.pose = toPose(ref.state.root);
  // nav_msgs/Odometry twists are expressed in the child frame.
  const Eigen::Vector3d v = ref.state.root.linear().transpose() * ref.velocity.head<3>();
  const Eigen::Vector3d w = ref.state.root.linear().transpose() * ref.velocity.segment<3>(3);
  body.twist.twist.linear.x = v.x(); body.twist.twist.linear.y = v.y(); body.twist.twist.linear.z = v.z();
  body.twist.twist.angular.x = w.x(); body.twist.twist.angular.y = w.y(); body.twist.twist.angular.z = w.z();
  sensor_msgs::JointState joints;
  joints.header = body.header; joints.name = info_.joint_names;
  for (int i = 0; i < ref.state.joints.size(); ++i)
  {
    joints.position.push_back(ref.state.joints[i]);
    joints.velocity.push_back(ref.velocity[6 + i]);
  }
  body_.publish(body); joints_.publish(joints);
  if (!execute) return;
  aerial_robot_msgs::FlightNav nav;
  nav.header = body.header;
  nav.target = aerial_robot_msgs::FlightNav::COG;
  nav.control_frame = aerial_robot_msgs::FlightNav::WORLD_FRAME;
  nav.pos_xy_nav_mode = nav.pos_z_nav_mode = nav.yaw_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
  nav.target_pos_x = ref.state.cog.translation().x();
  nav.target_pos_y = ref.state.cog.translation().y();
  nav.target_pos_z = ref.state.cog.translation().z();
  Eigen::Quaterniond q(ref.state.cog.linear());
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(q.x(), q.y(), q.z(), q.w())).getRPY(roll, pitch, yaw);
  nav.target_yaw = yaw;
  if (full_attitude_)
  {
    nav.roll_nav_mode = nav.pitch_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
    nav.target_roll = roll; nav.target_pitch = pitch;
  }
  // Position modes clear the old navigator's velocity feedforward. CoG pose
  // uses q_next; root twist remains available in the logical reference above.
  navigation_.publish(nav);
  if (!joints.name.empty()) joint_command_.publish(joints);
}
}
