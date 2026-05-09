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
  landing_or_halt_mode_(false)
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

  final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot", 1, &GimbalrotorMultilinkNavigator::targetBaselinkRotCallback, this);

  final_target_baselink_rpy_sub_ = nh_.subscribe("final_target_baselink_rpy", 1, &GimbalrotorMultilinkNavigator::targetBaselinkRPYCallback, this);

  prev_rotation_stamp_ = ros::Time::now().toSec();
  prev_joint_stamp_ = ros::Time::now().toSec();

  /*Takeoff / initialization: pitch joint should start from zero.*/
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

  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);

  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
  robot_model_->setCogDesireOrientation(rot);

  /*Reset returns the transformable part to safe zero angle.*/
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

  target_omega_.setValue(0, 0, 0);

  publishPitchJointCommand(landing_pitch_joint_angle_);

  ROS_INFO("[GimbalrotorMultilinkNavigator] halt: force pitch joint to zero and stop transformable motion.");
}

void GimbalrotorMultilinkNavigator::targetBaselinkRotCallback(
    const geometry_msgs::QuaternionStampedConstPtr& msg)
{
  /*Receive desired MAIN BODY orientation as quaternion.*/
  tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_rot_);

  /*Static attitude command, so target angular velocity is zero.*/
  target_omega_.setValue(0, 0, 0);

  /*Same special yaw handling as original gimbalrotor navigator.*/
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

  /*Static attitude command, so target angular velocity is zero.*/
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

      /*Limit rotation step to avoid sudden attitude command jump.*/
      if(fabs(angle) > baselink_rot_change_thresh_)
        {
          curr_target_baselink_rot_ *= tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);
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

  /*During landing or halt, compensation must stop. Otherwise the pitch joint may continue moving according to body pitch.*/
  if(landing_or_halt_mode_) return;

  if(ros::Time::now().toSec() - prev_joint_stamp_ < joint_cmd_pub_interval_)
    {
      return;
    }

  /*Extract current commanded MAIN BODY pitch.*/
  double body_roll, body_pitch, body_yaw;
  tf::Matrix3x3(curr_target_baselink_rot_).getRPY(body_roll,
                                                  body_pitch,
                                                  body_yaw);

  /*
   * Compensation idea: hand_world_pitch = body_pitch + pitch_joint_angle
   * To keep hand horizontal: hand_world_pitch = 0
   * Therefore: pitch_joint_angle = -body_pitch
   * But sign depends on your URDF joint axis.
   */
  double pitch_joint_cmd = pitch_joint_compensation_sign_ * body_pitch + pitch_joint_offset_;

  if(pitch_joint_cmd > pitch_joint_limit_)
    {
      pitch_joint_cmd = pitch_joint_limit_;
    }
  else if(pitch_joint_cmd < -pitch_joint_limit_)
    {
      pitch_joint_cmd = -pitch_joint_limit_;
    }

  publishPitchJointCommand(pitch_joint_cmd);

  prev_joint_stamp_ = ros::Time::now().toSec();
}

void GimbalrotorMultilinkNavigator::landingProcess()
{
  /*
   * When landing / force landing starts:
   *   1. Stop target angular velocity.
   *   2. Stop pitch compensation.
   *   3. Force pitch joint to zero.
   *   4. Make main body target level.
   *   5. Let the existing LAND_STATE / FORCE_LANDING_STATE continue.
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

          /*Force pitch joint / hand to zero.*/
          publishPitchJointCommand(landing_pitch_joint_angle_);

          /*Make main body attitude target level. Keep yaw, remove roll and pitch.*/
          double r, p, y;
          tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);

          final_target_baselink_rot_.setRPY(0, 0, y);

          level_flag_ = true;
        }
      else
        {
          /*Keep publishing zero pitch-joint command while landing. This makes sure the transformable joint does not stop halfway.*/
          if(ros::Time::now().toSec() - prev_joint_stamp_ > joint_cmd_pub_interval_)
            {
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
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorMultilinkNavigator,
                       aerial_robot_navigation::BaseNavigator);
