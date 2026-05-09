#include <gimbalrotor/model/gimbalrotor_multilink_robot_model.h>

#include <ros/ros.h>

#include <algorithm>
#include <sstream>

namespace
{
bool containsString(const std::string& text, const std::string& pattern)
{
  return text.find(pattern) != std::string::npos;
}
}

GimbalrotorMultilinkRobotModel::GimbalrotorMultilinkRobotModel(bool init_with_rosparam,
                                                               bool verbose,
                                                               double fc_t_min_thre,
                                                               double epsilon) :
  GimbalrotorRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon),
  has_movable_body_joints_(false),
  movable_body_joint_detection_done_(false)
{
}

bool GimbalrotorMultilinkRobotModel::isTargetBodyJointForMultilinkDetection(const std::string& joint_name) const
{
  if(containsString(joint_name, "pitch_joint"))
    return true;

  if(containsString(joint_name, "yaw_joint"))
    return true;

  if(containsString(joint_name, "roll_joint"))
    return true;

  return false;
}

void GimbalrotorMultilinkRobotModel::detectMovableBodySegments()
{
  movable_body_joint_names_.clear();
  movable_body_segment_names_.clear();

  {
    std::lock_guard<std::mutex> lock(body_links_rotation_mutex_);
    body_links_rotation_from_cog_.clear();
  }

  const KDL::Tree& tree = getTree();
  const KDL::SegmentMap& segments = tree.getSegments();

  for(const auto& segment_pair : segments)
    {
      const KDL::Segment& segment = segment_pair.second.segment;
      const KDL::Joint& joint = segment.getJoint();

      const std::string joint_name = joint.getName();
      const std::string segment_name = segment.getName();

      if(joint.getType() == KDL::Joint::None)
        continue;

        if(joint_name.empty() || joint_name == "NoName")
        continue;
      
      if(!isTargetBodyJointForMultilinkDetection(joint_name))
        {
          ROS_DEBUG_STREAM("GimbalrotorMultilinkRobotModel ignored movable joint ["
                           << joint_name
                           << " -> "
                           << segment_name
                           << "] because it is not pitch/yaw/roll joint.");
          continue;
        }

      movable_body_joint_names_.push_back(joint_name);
      movable_body_segment_names_.push_back(segment_name);
    }

  has_movable_body_joints_ = !movable_body_joint_names_.empty();

  {
    std::lock_guard<std::mutex> lock(body_links_rotation_mutex_);
    body_links_rotation_from_cog_.resize(movable_body_segment_names_.size());
  }

  movable_body_joint_detection_done_ = true;

  if(has_movable_body_joints_)
    {
      std::stringstream ss;
      ss << "GimbalrotorMultilinkRobotModel detected target body joints: ";

      for(size_t i = 0; i < movable_body_joint_names_.size(); ++i)
        {
          ss << "["
             << movable_body_joint_names_[i]
             << " -> "
             << movable_body_segment_names_[i]
             << "] ";
        }

      ROS_INFO_STREAM(ss.str());
      ROS_INFO_STREAM("Robot model mode: gimbalrotor + pitch/yaw/roll multilink body.");
    }
  else
    {
      ROS_INFO_STREAM("GimbalrotorMultilinkRobotModel detected no target body joints.");
      ROS_INFO_STREAM("Robot model mode: same as normal gimbalrotor.");
    }
}

void GimbalrotorMultilinkRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  GimbalrotorRobotModel::updateRobotModelImpl(joint_positions);

  if(!movable_body_joint_detection_done_)
    {
      detectMovableBodySegments();
    }

  if(!has_movable_body_joints_)
    {
      return;
    }

  KDL::TreeFkSolverPos_recursive fk_solver(getTree());

  KDL::Frame f_baselink;
  const int base_fk_result =
    fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());

  if(base_fk_result < 0)
    {
      ROS_WARN_STREAM_THROTTLE(
        1.0,
        "GimbalrotorMultilinkRobotModel: failed to get FK for base link ["
        << getBaselinkName()
        << "]");
      return;
    }

  /*
   * Construct CoG frame rotation.
   * Same formula as the original / Hydrus-like model: R_world_cog = R_world_baselink * R_cog_desired^{-1}
   * Then for a body segment: R_cog_body = R_world_cog^{-1} * R_world_body
   */
  const KDL::Rotation cog_frame =
    f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();

  /*
   * Update extra movable body-link rotations. These are NOT thrust links. 
   * Example: pitch_joint -> hand_assem_link
   */
  {
    std::lock_guard<std::mutex> lock(body_links_rotation_mutex_);

    for(size_t i = 0; i < movable_body_segment_names_.size(); ++i)
      {
        const std::string& segment_name = movable_body_segment_names_[i];

        KDL::Frame f_body_link;
        const int fk_result =
          fk_solver.JntToCart(joint_positions, f_body_link, segment_name);

        if(fk_result < 0)
          {
            ROS_WARN_STREAM_THROTTLE(
              1.0,
              "GimbalrotorMultilinkRobotModel: failed to get FK for movable body segment ["
              << segment_name
              << "]");
            continue;
          }

        /*
         * Movable body-link orientation relative to CoG.
         * Example:
         *   pitch_joint -> hand_assem_link
         *   R_cog_hand = R_world_cog^{-1} * R_world_hand
         */
        body_links_rotation_from_cog_[i] =
          cog_frame.Inverse() * f_body_link.M;
      }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(GimbalrotorMultilinkRobotModel, aerial_robot_model::RobotModel);