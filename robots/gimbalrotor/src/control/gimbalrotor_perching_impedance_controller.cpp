#include <gimbalrotor/control/gimbalrotor_perching_impedance_controller.h>

#include <pluginlib/class_list_macros.h>

#include <cmath>

namespace
{
const double PI = 3.14159265358979323846;
}

namespace aerial_robot_control
{

GimbalrotorPerchingImpedanceController::GimbalrotorPerchingImpedanceController()
  : GimbalrotorImpedanceController(),
    perching_enable_topic_for_constraint_("perching/enable"),
    perching_point_topic_("perching/point"),
    perching_branch_pose_topic_("perching/branch_pose"),
    perching_locked_pose_topic_("perching/locked_pose"),
    perching_enabled_for_constraint_(false),
    has_perching_point_(false),
    has_branch_pose_(false),
    has_locked_pose_(false),
    use_branch_pose_if_no_point_(true),
    require_perching_lock_(true),
    min_valid_radius_(0.05),
    max_pitch_delta_(0.78539816339),
    arc_pitch_sign_(1.0),
    locked_radius_(0.0),
    locked_x_side_(1.0)
{
  perching_point_world_.setValue(0.0, 0.0, 0.0);
  branch_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_rpy_.setValue(0.0, 0.0, 0.0);
  locked_radius_vec_world_.setValue(0.0, 0.0, 0.0);
}

void GimbalrotorPerchingImpedanceController::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  /*
   * First initialize the normal gimbalrotor impedance controller.
   *
   * This preserves normal impedance behavior.
   * This subclass only changes target injection while perching is active.
   */
  GimbalrotorImpedanceController::initialize(
      nh,
      nhp,
      robot_model,
      estimator,
      navigator,
      ctrl_loop_rate);

  perchingRosParamInit();

  perching_enable_sub_for_constraint_ =
      nh_.subscribe(
          perching_enable_topic_for_constraint_,
          1,
          &GimbalrotorPerchingImpedanceController::perchingEnableCallback,
          this);

  perching_point_sub_ =
      nh_.subscribe(
          perching_point_topic_,
          1,
          &GimbalrotorPerchingImpedanceController::perchingPointCallback,
          this);

  branch_pose_sub_ =
      nh_.subscribe(
          perching_branch_pose_topic_,
          1,
          &GimbalrotorPerchingImpedanceController::branchPoseCallback,
          this);

  /*
   * GimbalrotorPerchingNavigator publishes this as latched.
   * It contains:
   *   - locked robot position
   *   - locked robot RPY
   *
   * We use this to reconstruct the same pitch arc in the impedance controller.
   */
  locked_pose_sub_ =
      nh_.subscribe(
          perching_locked_pose_topic_,
          1,
          &GimbalrotorPerchingImpedanceController::lockedPoseCallback,
          this);

  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] initialized.");
  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] perching_enable_topic: "
                  << perching_enable_topic_for_constraint_);
  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] perching_point_topic: "
                  << perching_point_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] branch_pose_topic: "
                  << perching_branch_pose_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] locked_pose_topic: "
                  << perching_locked_pose_topic_);
}

void GimbalrotorPerchingImpedanceController::reset()
{
  GimbalrotorImpedanceController::reset();

  perching_enabled_for_constraint_ = false;
  has_perching_point_ = false;
  has_branch_pose_ = false;
  has_locked_pose_ = false;

  locked_radius_ = 0.0;
  locked_x_side_ = 1.0;

  perching_point_world_.setValue(0.0, 0.0, 0.0);
  branch_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_rpy_.setValue(0.0, 0.0, 0.0);
  locked_radius_vec_world_.setValue(0.0, 0.0, 0.0);

  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching state reset.");
}

void GimbalrotorPerchingImpedanceController::perchingRosParamInit()
{
  ros::NodeHandle imp_perch_nh(nh_, "controller/impedance/perching");

  /*
   * Defaults are intentionally the same topic names used by
   * GimbalrotorPerchingNavigator.
   */
  getParam<std::string>(
      imp_perch_nh,
      "perching_enable_topic",
      perching_enable_topic_for_constraint_,
      std::string("perching/enable"));

  getParam<std::string>(
      imp_perch_nh,
      "perching_point_topic",
      perching_point_topic_,
      std::string("perching/point"));

  getParam<std::string>(
      imp_perch_nh,
      "perching_branch_pose_topic",
      perching_branch_pose_topic_,
      std::string("perching/branch_pose"));

  getParam<std::string>(
      imp_perch_nh,
      "perching_locked_pose_topic",
      perching_locked_pose_topic_,
      std::string("perching/locked_pose"));

  getParam<bool>(
      imp_perch_nh,
      "use_branch_pose_if_no_point",
      use_branch_pose_if_no_point_,
      true);

  getParam<bool>(
      imp_perch_nh,
      "require_perching_lock",
      require_perching_lock_,
      true);

  /*
   * These must match the perching navigator parameters.
   *
   * If they are not explicitly set under controller/impedance/perching,
   * read the existing navigation parameters.
   */
  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(
      navi_nh,
      "perching_min_valid_radius",
      min_valid_radius_,
      0.05);

  getParam<double>(
      navi_nh,
      "perching_max_pitch_delta",
      max_pitch_delta_,
      0.78539816339);

  getParam<double>(
      navi_nh,
      "perching_arc_pitch_sign",
      arc_pitch_sign_,
      1.0);

  getParam<double>(
      imp_perch_nh,
      "perching_min_valid_radius",
      min_valid_radius_,
      min_valid_radius_);

  getParam<double>(
      imp_perch_nh,
      "perching_max_pitch_delta",
      max_pitch_delta_,
      max_pitch_delta_);

  getParam<double>(
      imp_perch_nh,
      "perching_arc_pitch_sign",
      arc_pitch_sign_,
      arc_pitch_sign_);

  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching_min_valid_radius: %.4f",
           min_valid_radius_);
  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching_max_pitch_delta deg: %.2f",
           max_pitch_delta_ * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching_arc_pitch_sign: %.2f",
           arc_pitch_sign_);
}

void GimbalrotorPerchingImpedanceController::perchingEnableCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
  perching_enabled_for_constraint_ = msg->data;

  if(!perching_enabled_for_constraint_)
    {
      ROS_WARN("[GimbalrotorPerchingImpedanceController] perching constraint DISABLED; use normal impedance.");
      return;
    }

  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching constraint ENABLED.");
}

void GimbalrotorPerchingImpedanceController::perchingPointCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg)
{
  perching_point_world_.setValue(
      msg->point.x,
      msg->point.y,
      msg->point.z);

  has_perching_point_ = true;
}

void GimbalrotorPerchingImpedanceController::branchPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  branch_pos_world_.setValue(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

  has_branch_pose_ = true;

  if(!has_perching_point_ && use_branch_pose_if_no_point_)
    {
      perching_point_world_ = branch_pos_world_;
    }
}

void GimbalrotorPerchingImpedanceController::lockedPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  poseMsgToTfPosRpy(*msg, locked_robot_pos_world_, locked_robot_rpy_);

  if(!has_perching_point_ && use_branch_pose_if_no_point_ && has_branch_pose_)
    {
      perching_point_world_ = branch_pos_world_;
    }

  locked_radius_vec_world_ =
      locked_robot_pos_world_ - perching_point_world_;

  locked_radius_ =
      norm2D(
          locked_radius_vec_world_.x(),
          locked_radius_vec_world_.z());

  if(locked_robot_pos_world_.x() - perching_point_world_.x() >= 0.0)
    {
      locked_x_side_ = 1.0;
    }
  else
    {
      locked_x_side_ = -1.0;
    }

  if(locked_radius_ < min_valid_radius_)
    {
      has_locked_pose_ = false;

      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingImpedanceController] locked pose received, "
          "but radius %.4f is too small. Waiting for valid lock.",
          locked_radius_);

      return;
    }

  has_locked_pose_ = true;

  ROS_WARN("[GimbalrotorPerchingImpedanceController] locked pose received.");
  ROS_WARN("[GimbalrotorPerchingImpedanceController] pivot: %.3f %.3f %.3f",
           perching_point_world_.x(),
           perching_point_world_.y(),
           perching_point_world_.z());
  ROS_WARN("[GimbalrotorPerchingImpedanceController] locked pos: %.3f %.3f %.3f",
           locked_robot_pos_world_.x(),
           locked_robot_pos_world_.y(),
           locked_robot_pos_world_.z());
  ROS_WARN("[GimbalrotorPerchingImpedanceController] locked pitch deg: %.2f",
           locked_robot_rpy_.y() * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingImpedanceController] X-Z radius: %.3f",
           locked_radius_);
}

bool GimbalrotorPerchingImpedanceController::hasValidPerchingConstraint() const
{
  if(!perching_enabled_for_constraint_)
    {
      return false;
    }

  if(require_perching_lock_ && !has_locked_pose_)
    {
      return false;
    }

  if(!has_perching_point_ &&
     !(use_branch_pose_if_no_point_ && has_branch_pose_))
    {
      return false;
    }

  if(locked_radius_ < min_valid_radius_)
    {
      return false;
    }

  return true;
}

void GimbalrotorPerchingImpedanceController::applyImpedanceOutputToNavigator(
    const tf::Vector3& original_target_pos,
    const tf::Vector3& original_target_rpy,
    const ImpedanceCoreOutput& output)
{
  /*
   * If not perching, keep the exact behavior of the normal impedance controller.
   */
  if(!hasValidPerchingConstraint())
    {
      GimbalrotorImpedanceController::applyImpedanceOutputToNavigator(
          original_target_pos,
          original_target_rpy,
          output);

      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingImpedanceController] using NORMAL impedance "
          "(perching constraint not valid).");

      return;
    }

  /*
   * Perching-compatible impedance:
   *
   * Normal perching navigator already gives a nominal target:
   *   original_target_pos = CoG position on pitch arc
   *   original_target_rpy.y = nominal pitch on pitch arc
   *
   * ImpedanceCore gives:
   *   output.rpy_offset_world(1) = soft pitch offset from external torque
   *
   * Correct compatible target:
   *   pitch_new = nominal_pitch + delta_pitch_impedance
   *   pos_new   = perching_arc_position(pitch_new)
   *
   * This prevents the bad case:
   *   pitch changes but CoG position does not move along the branch arc.
   */
  const double nominal_pitch =
      original_target_rpy.y();

  const double impedance_pitch_offset =
      output.rpy_offset_world(1);

  const double target_pitch =
      normalizeAngle(nominal_pitch + impedance_pitch_offset);

  tf::Vector3 modified_target_pos =
      computePerchingArcPositionFromPitch(
          target_pitch,
          original_target_pos);

  /*
   * Preserve normal branch-axis / world-Y target from the perching navigator.
   *
   * If later you enable Y impedance, allow only Y position offset here.
   * X and Z arbitrary impedance offsets are intentionally ignored during
   * perching, because X-Z must stay on the pitch arc.
   */
  modified_target_pos.setY(
      original_target_pos.y() + output.pos_offset_world(1));

  tf::Vector3 modified_target_rpy = original_target_rpy;

  /*
   * Roll and yaw can still use normal small impedance offsets if enabled.
   * For your current YAML, roll/yaw are disabled, so these are normally zero.
   */
  modified_target_rpy.setX(
      modified_target_rpy.x() + output.rpy_offset_world(0));

  modified_target_rpy.setY(target_pitch);

  modified_target_rpy.setZ(
      modified_target_rpy.z() + output.rpy_offset_world(2));

  navigator_->setTargetPos(modified_target_pos);
  navigator_->setTargetRPY(modified_target_rpy);

  ROS_WARN_THROTTLE(
      0.5,
      "[GimbalrotorPerchingImpedanceController] PERCHING impedance | "
      "nominal_pitch %.2f deg | d_pitch %.2f deg | target_pitch %.2f deg | "
      "target_pos %.3f %.3f %.3f",
      nominal_pitch * 180.0 / PI,
      impedance_pitch_offset * 180.0 / PI,
      target_pitch * 180.0 / PI,
      modified_target_pos.x(),
      modified_target_pos.y(),
      modified_target_pos.z());
}

tf::Vector3
GimbalrotorPerchingImpedanceController::computePerchingArcPositionFromPitch(
    double target_pitch,
    const tf::Vector3& original_target_pos) const
{
  /*
   * Same geometric idea as GimbalrotorPerchingNavigator::computeArcPositionFromPitch().
   *
   * Locked:
   *   pivot C
   *   robot position P0
   *   radius vector r0 = P0 - C
   *   pitch theta0
   *
   * Command:
   *   target pitch theta
   *
   * Compute:
   *   dtheta = theta - theta0
   *   r_des = RotY(dtheta) * r0
   *   P_des = C + r_des
   *
   * Use only X-Z radius because branch axis is world Y.
   */
  double delta_pitch =
      normalizeAngle(target_pitch - locked_robot_rpy_.y());

  delta_pitch =
      clamp(delta_pitch, -max_pitch_delta_, max_pitch_delta_);

  const double signed_delta =
      arc_pitch_sign_ * delta_pitch;

  tf::Vector3 locked_radius_xz(
      locked_radius_vec_world_.x(),
      0.0,
      locked_radius_vec_world_.z());

  double length_xz =
      norm2D(
          locked_radius_xz.x(),
          locked_radius_xz.z());

  if(length_xz < 1.0e-6)
    {
      locked_radius_xz.setX(locked_x_side_ * locked_radius_);
      locked_radius_xz.setY(0.0);
      locked_radius_xz.setZ(0.0);
      length_xz = locked_radius_;
    }

  if(length_xz < 1.0e-6)
    {
      locked_radius_xz.setX(locked_x_side_);
      locked_radius_xz.setY(0.0);
      locked_radius_xz.setZ(0.0);
      length_xz = 1.0;
    }

  locked_radius_xz.setX(
      locked_radius_xz.x() / length_xz * locked_radius_);
  locked_radius_xz.setY(0.0);
  locked_radius_xz.setZ(
      locked_radius_xz.z() / length_xz * locked_radius_);

  tf::Matrix3x3 rot(
      tf::createQuaternionFromRPY(
          0.0,
          signed_delta,
          0.0));

  tf::Vector3 rotated_radius =
      rot * locked_radius_xz;

  double rotated_length_xz =
      norm2D(
          rotated_radius.x(),
          rotated_radius.z());

  if(rotated_length_xz < 1.0e-6)
    {
      rotated_radius.setX(locked_x_side_ * locked_radius_);
      rotated_radius.setY(0.0);
      rotated_radius.setZ(0.0);
    }
  else
    {
      rotated_radius.setX(
          rotated_radius.x() / rotated_length_xz * locked_radius_);
      rotated_radius.setY(0.0);
      rotated_radius.setZ(
          rotated_radius.z() / rotated_length_xz * locked_radius_);
    }

  tf::Vector3 target_pos =
      perching_point_world_ + rotated_radius;

  /*
   * Preserve Y generated by GimbalrotorPerchingNavigator.
   *
   * The navigator already handles Y deadband/compliance. This controller should
   * not duplicate that logic.
   */
  target_pos.setY(original_target_pos.y());

  return target_pos;
}

double GimbalrotorPerchingImpedanceController::clamp(
    double value,
    double min_value,
    double max_value) const
{
  if(value < min_value)
    {
      return min_value;
    }

  if(value > max_value)
    {
      return max_value;
    }

  return value;
}

double GimbalrotorPerchingImpedanceController::normalizeAngle(
    double angle) const
{
  while(angle > PI)
    {
      angle -= 2.0 * PI;
    }

  while(angle < -PI)
    {
      angle += 2.0 * PI;
    }

  return angle;
}

double GimbalrotorPerchingImpedanceController::norm2D(
    double x,
    double z) const
{
  return std::sqrt(x * x + z * z);
}

void GimbalrotorPerchingImpedanceController::poseMsgToTfPosRpy(
    const geometry_msgs::PoseStamped& msg,
    tf::Vector3& pos,
    tf::Vector3& rpy) const
{
  pos.setValue(
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z);

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.pose.orientation, q);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  rpy.setValue(roll, pitch, yaw);
}

} // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_control::GimbalrotorPerchingImpedanceController,
    aerial_robot_control::ControlBase)