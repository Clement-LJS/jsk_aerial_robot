#pragma once
#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <gimbalrotor/pitch_only_perching_core.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <map>
#include <vector>

namespace aerial_robot_navigation
{
class GimbalrotorPitchOnlyPerchingNavigator : public GimbalrotorPerchingNavigator
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,double dt) override;
  void update() override;
  bool perchingAdmittanceTarget(const ros::Time&,double,const tf::Vector3&,perching_geometry::Pose&) const override { return false; }
protected:
  bool supportsPerchingMode(perching_geometry::Mode m) const override { return m==perching_geometry::Mode::NORMAL; }
  void selectPerchingMode(perching_geometry::Mode mode,bool enable) override;
  void relockCallback(const std_msgs::EmptyConstPtr&) override;
  bool tryLockPerching(const std::string&) override;
  void resetPerchingLock() override;
  void applyActivePerchingTarget() override;
  bool applyPerchingConstraint(aerial_robot_msgs::FlightNav&) override { return !core_.ownsPose(); }
  void applyManualPitchDelta(double delta) override;
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;
private:
  bool resolveModel();
  bool modelAt(double q,KDL::Frame& bc,KDL::Vector& cog) const;
  pitch_only::Sample sample() const;
  void jointCallback(const sensor_msgs::JointStateConstPtr& msg);
  void request(const std::string& action);
  void commitLock();
  void diagnostics();
  bool commandPose();
  pitch_only::Core core_;
  pitch_only::Sample feedback_;
  bool model_valid_=false, model_resolution_attempted_=false;
  double pitch_command_sign_=1, pitch_axis_alignment_threshold_=.90;
  double configured_lower_=pitch_only::unknown, configured_upper_=pitch_only::unknown;
  double configured_rate_=pitch_only::unknown;
  std::size_t joint_index_=0;
  double joint_receive_stamp_=0;
  std::string joint_name_,contact_link_,world_frame_,prefix_;
  ros::Time last_update_;
  std::vector<ros::Subscriber> subscribers_;
  ros::Publisher joints_pub_,pose_pub_,pivot_pub_,state_pub_;
  std::map<std::string,ros::Publisher> bool_pubs_,number_pubs_;
};
}
