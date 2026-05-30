// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_multilink_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

GimbalrotorMultilinkNavigator::GimbalrotorMultilinkNavigator():
  BaseNavigator(),
  prev_rotation_stamp_(0.0),
  prev_joint_stamp_(0.0),
  baselink_rot_change_thresh_(0.02),
  baselink_rot_pub_interval_(0.1),
  joint_cmd_pub_interval_(0.02),
  pitch_joint_compensation_sign_(1.0),
  pitch_joint_offset_(0.0),
  pitch_joint_limit_(1.57),
  takeoff_pitch_joint_angle_(0.0),
  landing_pitch_joint_angle_(0.0),
  pitch_joint_land_thresh_(0.085),
  pitch_joint_name_("pitch_joint"),
  eq_cog_world_(false),
  keep_hand_horizontal_(true),
  level_flag_(false),
  landing_or_halt_mode_(false),

  /*
   * New parameters.
   *
   * These prevent the pitch joint / hand link from moving much faster
   * than the aerial body can follow.
   */
  pitch_joint_max_velocity_(0.25),
  pitch_joint_lpf_rate_(0.2),
  pitch_joint_use_estimated_body_pitch_(false),
  prev_pitch_joint_cmd_(0.0),
  prev_pitch_joint_cmd_initialized_(false),

  interaction_mode_(NORMAL_FLIGHT),
  pitch_joint_compliance_offset_(0.0),
  pitch_joint_compliance_offset_limit_(0.12),
  branch_alignment_offset_(0.0),
  cutting_feed_offset_(0.0),
  use_pitch_joint_compliance_in_cutting_(true)

{
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
}

void GimbalrotorMultilinkNavigator::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    double loop_du)
{
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  target_baselink_rpy_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);

  final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot",
                                                 1,
                                                 &GimbalrotorMultilinkNavigator::targetBaselinkRotCallback,
                                                 this);

  final_target_baselink_rpy_sub_ = nh_.subscribe("final_target_baselink_rpy",
                                                 1,
                                                 &GimbalrotorMultilinkNavigator::targetBaselinkRPYCallback,
                                                 this);

  interaction_mode_sub_ =
    nh_.subscribe("interaction_mode",
                  1,
                  &GimbalrotorMultilinkNavigator::interactionModeCallback,
                  this);

  pitch_joint_compliance_offset_sub_ =
    nh_.subscribe("pitch_joint_compliance_offset",
                  1,
                  &GimbalrotorMultilinkNavigator::pitchJointComplianceOffsetCallback,
                  this);



  
  prev_rotation_stamp_ = ros::Time::now().toSec();
  prev_joint_stamp_ = ros::Time::now().toSec();

  /*
   * Takeoff / initialization:
   * pitch joint should start from safe zero or configured takeoff angle.
   */
  prev_pitch_joint_cmd_ = takeoff_pitch_joint_angle_;
  prev_pitch_joint_cmd_initialized_ = true;

  publishPitchJointCommand(takeoff_pitch_joint_angle_);
}

void GimbalrotorMultilinkNavigator::update()
{
  BaseNavigator::update();

  baselinkRotationProcess();

  landingProcess();

  pitchLinkCompensationProcess();
}

void GimbalrotorMultilinkNavigator::reset()
{
  BaseNavigator::reset();

  eq_cog_world_ = false;
  level_flag_ = false;
  landing_or_halt_mode_ = false;

  interaction_mode_ = NORMAL_FLIGHT;
  pitch_joint_compliance_offset_ = 0.0;
  
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);

  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
  robot_model_->setCogDesireOrientation(rot);

  /*
   * Reset pitch joint command filter memory.
   */
  prev_pitch_joint_cmd_ = takeoff_pitch_joint_angle_;
  prev_pitch_joint_cmd_initialized_ = true;

  /*
   * Reset returns the transformable part to safe zero angle.
   */
  publishPitchJointCommand(takeoff_pitch_joint_angle_);
}

void GimbalrotorMultilinkNavigator::halt()
{
  /*
   * Safety halt:
   *
   * 1. Stop pitch compensation.
   * 2. Stop target angular velocity.
   * 3. Force pitch joint / hand to zero.
   *
   * This prevents transformable parts from continuing to move.
   */
  landing_or_halt_mode_ = true;
  level_flag_ = true;

  interaction_mode_ = NORMAL_FLIGHT;
  pitch_joint_compliance_offset_ = 0.0;
  
  target_omega_.setValue(0, 0, 0);

  prev_pitch_joint_cmd_ = landing_pitch_joint_angle_;
  prev_pitch_joint_cmd_initialized_ = true;

  publishPitchJointCommand(landing_pitch_joint_angle_);

  ROS_INFO("[GimbalrotorMultilinkNavigator] halt: force pitch joint to zero and stop transformable motion.");
}

void GimbalrotorMultilinkNavigator::targetBaselinkRotCallback(
    const geometry_msgs::QuaternionStampedConstPtr& msg)
{
  /*
   * Receive desired MAIN BODY orientation as quaternion.
   */
  tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_rot_);

  /*
   * Static attitude command, so target angular velocity is zero.
   */
  target_omega_.setValue(0, 0, 0);

  /*
   * Same special yaw handling as original gimbalrotor navigator.
   */
  if(getTargetRPY().z() != 0)
    {
      curr_target_baselink_rot_.setRPY(0, 0, getTargetRPY().z());
      eq_cog_world_ = true;
    }
}

void GimbalrotorMultilinkNavigator::targetBaselinkRPYCallback(
    const geometry_msgs::Vector3StampedConstPtr& msg)
{
  /*
   * Receive desired MAIN BODY roll / pitch / yaw.
   *
   * msg->vector.x = main body roll
   * msg->vector.y = main body pitch
   * msg->vector.z = main body yaw
   */
  final_target_baselink_rot_.setRPY(msg->vector.x,
                                    msg->vector.y,
                                    msg->vector.z);

  /*
   * Static attitude command, so target angular velocity is zero.
   */
  target_omega_.setValue(0, 0, 0);
}

void GimbalrotorMultilinkNavigator::naviCallback(
    const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  BaseNavigator::naviCallback(msg);

  if(msg->roll_nav_mode == 2)
    {
      setTargetRoll(msg->target_roll);
    }

  if(msg->pitch_nav_mode == 2)
    {
      setTargetPitch(msg->target_pitch);
    }

  if(landing_or_halt_mode_)
    {
      return;
    }

  /*
   * Multilink adaptation:
   *
   * roll / pitch command is interpreted as MAIN BODY attitude.
   * Pitch link / hand compensation is handled separately.
   */
  if(msg->roll_nav_mode == 2 || msg->pitch_nav_mode == 2)
    {
      tf::Vector3 target_rpy = getTargetRPY();

      final_target_baselink_rot_.setRPY(target_rpy.x(),
                                        target_rpy.y(),
                                        target_rpy.z());
    }
}

void GimbalrotorMultilinkNavigator::baselinkRotationProcess()
{
  if(curr_target_baselink_rot_ == final_target_baselink_rot_) return;

  if(ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
    {
      tf::Quaternion delta_q = curr_target_baselink_rot_.inverse() * final_target_baselink_rot_;
      double angle = delta_q.getAngle();

      if(angle > M_PI) angle -= 2 * M_PI;

      /*
       * Limit rotation step to avoid sudden attitude command jump.
       */
      if(fabs(angle) > baselink_rot_change_thresh_)
        {
          curr_target_baselink_rot_ *= tf::Quaternion(delta_q.getAxis(),
                                                      fabs(angle) / angle * baselink_rot_change_thresh_);
        }
      else
        {
          curr_target_baselink_rot_ = final_target_baselink_rot_;
        }

      KDL::Rotation rot;
      tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
      robot_model_->setCogDesireOrientation(rot);

      spinal::DesireCoord desire_msg;

      double r, p, y;
      tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);

      desire_msg.roll = r;
      desire_msg.pitch = p;
      desire_msg.yaw = y;

      target_baselink_rpy_pub_.publish(desire_msg);

      prev_rotation_stamp_ = ros::Time::now().toSec();
    }
}

void GimbalrotorMultilinkNavigator::pitchLinkCompensationProcess()
{
  if(!keep_hand_horizontal_) return;

  /*
   * During landing or halt, compensation must stop.
   * Otherwise the pitch joint may continue moving according to body pitch.
   */
  if(landing_or_halt_mode_) return;

  const double now = ros::Time::now().toSec();
  const double dt = now - prev_joint_stamp_;

  if(dt < joint_cmd_pub_interval_)
    {
      return;
    }

  /*
   * Get body pitch.
   *
   * Old behavior:
   *   Use curr_target_baselink_rot_, which is the commanded target body attitude.
   *
   * Problem:
   *   The pitch joint servo can move immediately according to the target,
   *   while the real drone body attitude is still lagging behind.
   *
   * New behavior:
   *   Optionally use estimated actual body attitude.
   *   This makes the hand compensate the real current body pitch.
   */
  double body_roll = 0.0;
  double body_pitch = 0.0;
  double body_yaw = 0.0;

  if(pitch_joint_use_estimated_body_pitch_)
    {
      tf::Matrix3x3 actual_body_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
      actual_body_rot.getRPY(body_roll, body_pitch, body_yaw);
    }
  else
    {
      tf::Matrix3x3(curr_target_baselink_rot_).getRPY(body_roll,
                                                      body_pitch,
                                                      body_yaw);
    }


 /*
  * Compensation idea:
  *
  *   hand_world_pitch = body_pitch + pitch_joint_angle
  *
  * To keep the hand/branch orientation constant:
  *
  *   pitch_joint_angle compensates body_pitch.
  *
  * This is needed during cutting too:
  *   - body pitches for cutting
  *   - pitch joint moves oppositely
  *   - hand link keeps the same world orientation / parallel to branch
  */
  double raw_pitch_joint_cmd =
      pitch_joint_compensation_sign_ * body_pitch + pitch_joint_offset_;

  if(interaction_mode_ == CUTTING_COMPLIANCE)
    {
     /*
      * During cutting, keep the hand orientation fixed relative to the branch.
      * Do NOT add pitch_joint_compliance_offset here.
      * Pitch impedance is applied to body pitch target in the controller.
      *
      * branch_alignment_offset can be used only if the branch is not horizontal.
      */
      raw_pitch_joint_cmd += branch_alignment_offset_;
    }


  /*
   * Apply joint limit.
   */
  if(raw_pitch_joint_cmd > pitch_joint_limit_)
    {
      raw_pitch_joint_cmd = pitch_joint_limit_;
    }
  else if(raw_pitch_joint_cmd < -pitch_joint_limit_)
    {
      raw_pitch_joint_cmd = -pitch_joint_limit_;
    }

  /*
   * Initialize command memory.
   */
  if(!prev_pitch_joint_cmd_initialized_)
    {
      prev_pitch_joint_cmd_ = getCurrentPitchJointPosition();
      prev_pitch_joint_cmd_initialized_ = true;
    }

  /*
   * Low-pass filter.
   *
   * filtered_cmd =
   *   previous_cmd + alpha * (raw_cmd - previous_cmd)
   *
   * alpha small  -> smoother/slower
   * alpha large  -> faster
   */
  double filtered_pitch_joint_cmd =
      prev_pitch_joint_cmd_
      + pitch_joint_lpf_rate_ * (raw_pitch_joint_cmd - prev_pitch_joint_cmd_);

  /*
   * Velocity / rate limit.
   *
   * This is the most important part for your problem.
   * It prevents the hand servo command from moving faster than the drone body.
   */
  double max_step = pitch_joint_max_velocity_ * dt;

  if(max_step < 0.0)
    {
      max_step = 0.0;
    }

  double diff = filtered_pitch_joint_cmd - prev_pitch_joint_cmd_;

  if(diff > max_step)
    {
      diff = max_step;
    }
  else if(diff < -max_step)
    {
      diff = -max_step;
    }

  double pitch_joint_cmd = prev_pitch_joint_cmd_ + diff;

  /*
   * Apply joint limit again after filtering/rate limiting.
   */
  if(pitch_joint_cmd > pitch_joint_limit_)
    {
      pitch_joint_cmd = pitch_joint_limit_;
    }
  else if(pitch_joint_cmd < -pitch_joint_limit_)
    {
      pitch_joint_cmd = -pitch_joint_limit_;
    }

  publishPitchJointCommand(pitch_joint_cmd);

  prev_pitch_joint_cmd_ = pitch_joint_cmd;
  prev_joint_stamp_ = now;

  ROS_DEBUG_STREAM_THROTTLE(
      0.5,
      "[GimbalrotorMultilinkNavigator] pitch compensation"
      << " use_estimated: " << pitch_joint_use_estimated_body_pitch_
      << " body_pitch: " << body_pitch
      << " raw_cmd: " << raw_pitch_joint_cmd
      << " filtered_cmd: " << filtered_pitch_joint_cmd
      << " final_cmd: " << pitch_joint_cmd
      << " dt: " << dt);
}

void GimbalrotorMultilinkNavigator::landingProcess()
{
  /*
   * When landing / force landing starts:
   *
   * 1. Stop target angular velocity.
   * 2. Stop pitch compensation.
   * 3. Force pitch joint to zero.
   * 4. Make main body target level.
   * 5. Let the existing LAND_STATE / FORCE_LANDING_STATE continue.
   */

  const bool landing =
      getForceLandingFlag() ||
      getNaviState() == LAND_STATE ||
      getNaviState() == FORCE_LANDING_STATE;

  if(landing)
    {
      target_omega_.setValue(0, 0, 0);

      landing_or_halt_mode_ = true;

      if(!level_flag_)
        {
          ROS_INFO("[GimbalrotorMultilinkNavigator] landing: force pitch joint to zero and stop transformable compensation.");

          /*
           * Force pitch joint / hand to zero.
           */
          prev_pitch_joint_cmd_ = landing_pitch_joint_angle_;
          prev_pitch_joint_cmd_initialized_ = true;

          publishPitchJointCommand(landing_pitch_joint_angle_);

          /*
           * Make main body attitude target level.
           * Keep yaw, remove roll and pitch.
           */
          double r, p, y;
          tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);

          final_target_baselink_rot_.setRPY(0, 0, y);

          level_flag_ = true;
        }
      else
        {
          /*
           * Keep publishing zero pitch-joint command while landing.
           * This makes sure the transformable joint does not stop halfway.
           */
          if(ros::Time::now().toSec() - prev_joint_stamp_ > joint_cmd_pub_interval_)
            {
              prev_pitch_joint_cmd_ = landing_pitch_joint_angle_;
              prev_pitch_joint_cmd_initialized_ = true;

              publishPitchJointCommand(landing_pitch_joint_angle_);
              prev_joint_stamp_ = ros::Time::now().toSec();
            }
        }

      double pitch_joint_pos = getCurrentPitchJointPosition();

      if(fabs(pitch_joint_pos - landing_pitch_joint_angle_) < pitch_joint_land_thresh_)
        {
          ROS_DEBUG("[GimbalrotorMultilinkNavigator] pitch joint is near landing angle.");
        }

      return;
    }

  if(landing_or_halt_mode_)
    {
      landing_or_halt_mode_ = false;
      level_flag_ = false;
    }
}

void GimbalrotorMultilinkNavigator::publishPitchJointCommand(double pitch_joint_cmd)
{
  sensor_msgs::JointState joint_msg;

  joint_msg.header.stamp = ros::Time::now();

  joint_msg.name.push_back(pitch_joint_name_);
  joint_msg.position.push_back(pitch_joint_cmd);

  joint_control_pub_.publish(joint_msg);
}

double GimbalrotorMultilinkNavigator::getCurrentPitchJointPosition()
{
  /*
   * Read current pitch joint position from robot model.
   */
  const auto joint_state =
      robot_model_->kdlJointToMsg(robot_model_->getJointPositions());

  for(int i = 0; i < joint_state.position.size(); i++)
    {
      if(joint_state.name[i] == pitch_joint_name_)
        {
          return joint_state.position[i];
        }
    }

  ROS_WARN_THROTTLE(1.0,
                    "[GimbalrotorMultilinkNavigator] pitch joint name [%s] not found in robot model. Check navigation/pitch_joint_name.",
                    pitch_joint_name_.c_str());

  return landing_pitch_joint_angle_;
}

void GimbalrotorMultilinkNavigator::interactionModeCallback(const std_msgs::UInt8ConstPtr& msg)
{
  if(msg->data > CUTTING_COMPLIANCE)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorMultilinkNavigator] invalid interaction_mode: %u",
          static_cast<unsigned int>(msg->data));
      return;
    }

  const uint8_t prev_mode = interaction_mode_;
  interaction_mode_ = msg->data;

  /*
   * If leaving cutting mode, remove pitch-joint compliance offset.
   * This prevents old cutting compliance from remaining during flight/perching.
   */

  if(interaction_mode_ != CUTTING_COMPLIANCE)
  {
    pitch_joint_compliance_offset_ = 0.0;
  }
  
  ROS_WARN_THROTTLE(
      1.0,
      "[GimbalrotorMultilinkNavigator] interaction_mode: %u",
      static_cast<unsigned int>(interaction_mode_));
}

void GimbalrotorMultilinkNavigator::pitchJointComplianceOffsetCallback(
    const std_msgs::Float64ConstPtr& msg)
{
  /*
   * Ignore pitch-joint compliance unless cutting mode is active.
   * During normal flight/perching, the hand must stay strictly horizontal.
   */
  if(interaction_mode_ != CUTTING_COMPLIANCE)
    {
      pitch_joint_compliance_offset_ = 0.0;
      return;
    }

  double offset = msg->data;

  if(offset > pitch_joint_compliance_offset_limit_)
    {
      offset = pitch_joint_compliance_offset_limit_;
    }
  else if(offset < -pitch_joint_compliance_offset_limit_)
    {
      offset = -pitch_joint_compliance_offset_limit_;
    }

  pitch_joint_compliance_offset_ = offset;
}

void GimbalrotorMultilinkNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1);
  getParam<double>(navi_nh, "joint_cmd_pub_interval", joint_cmd_pub_interval_, 0.02);
  getParam<std::string>(navi_nh, "pitch_joint_name", pitch_joint_name_, std::string("pitch_joint"));

  /*
   * If the hand tilts in the wrong direction during normal navigation,
   * change this parameter to -1.0 in YAML.
   */
  getParam<double>(navi_nh, "pitch_joint_compensation_sign", pitch_joint_compensation_sign_, 1.0);
  getParam<double>(navi_nh, "pitch_joint_offset", pitch_joint_offset_, 0.0);
  getParam<double>(navi_nh, "pitch_joint_limit", pitch_joint_limit_, 1.57);
  getParam<double>(navi_nh, "takeoff_pitch_joint_angle", takeoff_pitch_joint_angle_, 0.0);
  getParam<double>(navi_nh, "landing_pitch_joint_angle", landing_pitch_joint_angle_, 0.0);
  getParam<double>(navi_nh, "pitch_joint_land_thresh", pitch_joint_land_thresh_, 0.085);
  getParam<bool>(navi_nh, "keep_hand_horizontal", keep_hand_horizontal_, true);

  getParam<double>(navi_nh, "pitch_joint_compliance_offset_limit", pitch_joint_compliance_offset_limit_, 0.12);
  getParam<double>(navi_nh, "branch_alignment_offset", branch_alignment_offset_, 0.0);
  getParam<double>(navi_nh, "cutting_feed_offset", cutting_feed_offset_, 0.0);
  getParam<bool>(navi_nh, "use_pitch_joint_compliance_in_cutting", use_pitch_joint_compliance_in_cutting_, true);
  
  /*
   * New parameters for smooth pitch joint transformation.
   */
  getParam<double>(navi_nh, "pitch_joint_max_velocity", pitch_joint_max_velocity_, 0.25);
  getParam<double>(navi_nh, "pitch_joint_lpf_rate", pitch_joint_lpf_rate_, 0.2);
  getParam<bool>(navi_nh, "pitch_joint_use_estimated_body_pitch", pitch_joint_use_estimated_body_pitch_, false);

  /*
   * Safety clamp.
   */
  if(pitch_joint_max_velocity_ < 0.0)
    {
      pitch_joint_max_velocity_ = 0.0;
    }

  if(pitch_joint_lpf_rate_ < 0.0)
    {
      pitch_joint_lpf_rate_ = 0.0;
    }

  if(pitch_joint_lpf_rate_ > 1.0)
    {
      pitch_joint_lpf_rate_ = 1.0;
    }

  if(pitch_joint_compliance_offset_limit_ < 0.0)
    {
      pitch_joint_compliance_offset_limit_ = 0.0;
    }

  if(branch_alignment_offset_ > pitch_joint_limit_)
    {
      branch_alignment_offset_ = pitch_joint_limit_;
    }
  else if(branch_alignment_offset_ < -pitch_joint_limit_)
    {
      branch_alignment_offset_ = -pitch_joint_limit_;
    }

  if(cutting_feed_offset_ > pitch_joint_limit_)
    {
      cutting_feed_offset_ = pitch_joint_limit_;
    }
  else if(cutting_feed_offset_ < -pitch_joint_limit_)
    {
      cutting_feed_offset_ = -pitch_joint_limit_;
    }
  
  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] baselink_rot_change_thresh: "
                  << baselink_rot_change_thresh_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] baselink_rot_pub_interval: "
                  << baselink_rot_pub_interval_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] joint_cmd_pub_interval: "
                  << joint_cmd_pub_interval_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] pitch_joint_max_velocity: "
                  << pitch_joint_max_velocity_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] pitch_joint_lpf_rate: "
                  << pitch_joint_lpf_rate_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] pitch_joint_use_estimated_body_pitch: "
                  << pitch_joint_use_estimated_body_pitch_);

  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] pitch_joint_compliance_offset_limit: "
		  << pitch_joint_compliance_offset_limit_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] branch_alignment_offset: "
		  << branch_alignment_offset_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] cutting_feed_offset: "
		  << cutting_feed_offset_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkNavigator] use_pitch_joint_compliance_in_cutting: "
		  << use_pitch_joint_compliance_in_cutting_);






}

/*
 * Plugin registration
 */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorMultilinkNavigator,
                       aerial_robot_navigation::BaseNavigator);
