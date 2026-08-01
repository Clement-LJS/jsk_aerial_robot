#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>

#include <XmlRpcException.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace mujoco_ros_control
{
namespace
{
tf::Vector3 xmlRpcVector3(const XmlRpc::XmlRpcValue& value, const std::string& key)
{
  if(value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3)
    {
      throw std::runtime_error(key + " must be a 3-element array");
    }

  return tf::Vector3(static_cast<double>(value[0]), static_cast<double>(value[1]), static_cast<double>(value[2]));
}

bool finiteVector(const tf::Vector3& value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

bool normalizeVector(tf::Vector3& value)
{
  const double length = value.length();
  if(!std::isfinite(length) || length <= 1e-9)
    {
      return false;
    }
  value /= length;
  return true;
}

std::string objectName(mjModel* model, const int type, const int id)
{
  const char* name = mj_id2name(model, static_cast<mjtObj>(type), id);
  return name ? std::string(name) : std::string();
}

tf::Vector3 bodyPosition(const mjData* data, const int body_id)
{
  return tf::Vector3(data->xipos[3 * body_id + 0], data->xipos[3 * body_id + 1], data->xipos[3 * body_id + 2]);
}

tf::Vector3 sitePosition(const mjData* data, const int site_id)
{
  return tf::Vector3(data->site_xpos[3 * site_id + 0], data->site_xpos[3 * site_id + 1], data->site_xpos[3 * site_id + 2]);
}

void publishBool(ros::Publisher& pub, const ros::Time& time, const bool value)
{
  std_msgs::Bool msg;
  msg.data = value;
  pub.publish(msg);
}

void publishFloat(ros::Publisher& pub, const double value)
{
  std_msgs::Float64 msg;
  msg.data = value;
  pub.publish(msg);
}

void publishInt(ros::Publisher& pub, const int value)
{
  std_msgs::Int32 msg;
  msg.data = value;
  pub.publish(msg);
}
}  // namespace

bool AerialRobotHWSim::init(const std::string& robot_namespace,
                            ros::NodeHandle model_nh,
                            mjModel* mujoco_model,
                            mjData* mujoco_data)
{
  if(!DefaultRobotHWSim::init(robot_namespace, model_nh, mujoco_model, mujoco_data))
    {
      return false;
    }

  rotor_list_.clear();

  float mass = 0.0;
  for(int i = 0; i < mujoco_model_->nbody; i++)
    {
      mass += mujoco_model_->body_mass[i];
    }
  ROS_INFO_STREAM("[mujoco] robot mass is " << mass);

  int motor_num = 0;
  for(int i = 0; i < mujoco_model_->nu; i++)
    {
      const std::string actuator_name = objectName(mujoco_model_, mjOBJ_ACTUATOR, i);
      if(actuator_name.find("rotor") != std::string::npos)
        {
          rotor_list_.push_back(actuator_name);
          motor_num++;
        }
    }

  XmlRpc::XmlRpcValue all_servos_params;
  model_nh.getParam("servo_controller", all_servos_params);
  const std::string init_value_param_name = "init_value";
  for(auto servo_group_params: all_servos_params)
    {
      if(servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        continue;
      for(auto servo_params : servo_group_params.second)
        {
          if(servo_params.first.find("controller") != std::string::npos)
            {
              const std::string servo_name = static_cast<std::string>(servo_params.second["name"]);
              double init_value = 0.0;

              if(!servo_group_params.second.hasMember("simulation") && !servo_params.second.hasMember("simulation"))
                {
                  ROS_ERROR("please set mujoco servo parameters for %s, using sub namespace 'simulation:'", std::string(servo_params.second["name"]).c_str());
                  continue;
                }

              if(!servo_params.second.hasMember("simulation") ||
                 (servo_params.second.hasMember("simulation") && !servo_params.second["simulation"].hasMember(init_value_param_name)))
                {
                  if(!servo_group_params.second["simulation"].hasMember(init_value_param_name))
                    {
                      ROS_ERROR("can not find '%s' gazebo paramter for servo %s", init_value_param_name.c_str(), std::string(servo_params.second["name"]).c_str());
                      return false;
                    }
                  init_value = static_cast<double>(servo_group_params.second["simulation"][init_value_param_name]);
                }
              else
                {
                  init_value = static_cast<double>(servo_params.second["simulation"][init_value_param_name]);
                }
              control_input_.at(mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, servo_name.c_str())) = init_value;
            }
        }
    }

  spinal_interface_.init(model_nh, rotor_list_.size());
  registerInterface(&spinal_interface_);

  ros::NodeHandle simulation_nh(model_nh, "simulation");
  simulation_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01);
  simulation_nh.param("ground_truth_pos_noise", ground_truth_pos_noise_, 0.0);
  simulation_nh.param("ground_truth_vel_noise", ground_truth_vel_noise_, 0.0);
  simulation_nh.param("ground_truth_rot_noise", ground_truth_rot_noise_, 0.0);
  simulation_nh.param("ground_truth_angular_noise", ground_truth_angular_noise_, 0.0);
  simulation_nh.param("ground_truth_rot_drift", ground_truth_rot_drift_, 0.0);
  simulation_nh.param("ground_truth_vel_drift", ground_truth_vel_drift_, 0.0);
  simulation_nh.param("ground_truth_angular_drift", ground_truth_angular_drift_, 0.0);
  simulation_nh.param("ground_truth_rot_drift_frequency", ground_truth_rot_drift_frequency_, 0.0);
  simulation_nh.param("ground_truth_vel_drift_frequency", ground_truth_vel_drift_frequency_, 0.0);
  simulation_nh.param("ground_truth_angular_drift_frequency", ground_truth_angular_drift_frequency_, 0.0);
  simulation_nh.param("mocap_pub_rate", mocap_pub_rate_, 0.01);
  simulation_nh.param("mocap_pos_noise", mocap_pos_noise_, 0.001);
  simulation_nh.param("mocap_rot_noise", mocap_rot_noise_, 0.001);

  ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
  mocap_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);
  last_ground_truth_time_ = ros::Time(0);
  last_mocap_time_ = ros::Time(0);

  if(!initCutting(model_nh))
    {
      return false;
    }
  return true;
}

void AerialRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
{
  const int fc_id = mj_name2id(mujoco_model_, mjOBJ_SITE, "fc");
  mjtNum* site_xpos = mujoco_data_->site_xpos;
  mjtNum* site_xmat = mujoco_data_->site_xmat;
  tf::Matrix3x3 fc_rot_mat(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                           site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                           site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
  tf::Quaternion fc_quat;
  fc_rot_mat.getRotation(fc_quat);

  tf::Vector3 acc, gyro, mag;
  for(int i = 0; i < mujoco_model_->nsensor; i++)
    {
      const std::string sensor_name = objectName(mujoco_model_, mjOBJ_SENSOR, i);
      if(sensor_name == "acc")
        {
          for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
            acc[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
        }
      if(sensor_name == "gyro")
        {
          for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
            gyro[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
        }
      if(sensor_name == "mag")
        {
          for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
            mag[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
        }
    }

  spinal_interface_.setImuValue(acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
  spinal_interface_.setMagValue(mag.x(), mag.y(), mag.z());
  spinal_interface_.stateEstimate();

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = "fc";
  odom_msg.pose.pose.position.x = site_xpos[3 * fc_id + 0];
  odom_msg.pose.pose.position.y = site_xpos[3 * fc_id + 1];
  odom_msg.pose.pose.position.z = site_xpos[3 * fc_id + 2];
  odom_msg.pose.pose.orientation.x = fc_quat.x();
  odom_msg.pose.pose.orientation.y = fc_quat.y();
  odom_msg.pose.pose.orientation.z = fc_quat.z();
  odom_msg.pose.pose.orientation.w = fc_quat.w();

  spinal_interface_.setGroundTruthStates(fc_quat.x(), fc_quat.y(), fc_quat.z(), fc_quat.w(),
                                         gyro.x(), gyro.y(), gyro.z());

  if((time - last_ground_truth_time_).toSec() >= ground_truth_pub_rate_)
    {
      ground_truth_pub_.publish(odom_msg);
      last_ground_truth_time_ = time;
    }

  if((time - last_mocap_time_).toSec() >= mocap_pub_rate_)
    {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = time;
      pose_msg.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(mocap_pos_noise_);
      pose_msg.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(mocap_pos_noise_);
      pose_msg.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(mocap_pos_noise_);

      tf::Quaternion q_delta;
      q_delta.setRPY(gazebo::gaussianKernel(mocap_rot_noise_),
                     gazebo::gaussianKernel(mocap_rot_noise_),
                     gazebo::gaussianKernel(mocap_rot_noise_));
      const tf::Quaternion q_noise = fc_quat * q_delta;
      pose_msg.pose.orientation.x = q_noise.x();
      pose_msg.pose.orientation.y = q_noise.y();
      pose_msg.pose.orientation.z = q_noise.z();
      pose_msg.pose.orientation.w = q_noise.w();

      mocap_pub_.publish(pose_msg);
      last_mocap_time_ = time;
    }

  DefaultRobotHWSim::read(time, period);
}

void AerialRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
{
  for(int i = 0; i < spinal_interface_.getMotorNum(); i++)
    {
      const int rotor_id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, rotor_list_.at(i).c_str());
      control_input_.at(rotor_id) = spinal_interface_.getForce(i);
    }

  tf::Vector3 applied_force_world(0.0, 0.0, 0.0);
  tf::Vector3 torque_cog_world(0.0, 0.0, 0.0);
  tf::Vector3 torque_pivot_world(0.0, 0.0, 0.0);
  aerial_robot_simulation::MujocoCuttingForceModel::Output model_output;
  double arc_angle = 0.0;
  double locked_pivot_error = std::numeric_limits<double>::infinity();

  if(cutting_state_.enabled)
    {
      if(cutting_state_.saw_body_id >= 0)
        {
          for(int axis = 0; axis < 6; ++axis)
            mujoco_data_->xfrc_applied[6 * cutting_state_.saw_body_id + axis] = 0.0;
        }

      if(cutting_state_.reset_requested)
        {
          resetCuttingState();
          cutting_state_.reset_requested = false;
        }

      if(cutting_state_.has_locked_pivot)
        {
          locked_pivot_error = (cutting_state_.locked_pivot_world - cutting_state_.configured_pivot_world).length();
        }

      const bool ids_ready = cutting_state_.base_body_id >= 0 && cutting_state_.saw_body_id >= 0 && cutting_state_.contact_site_id >= 0;
      cutting_state_.valid = cutting_state_.model_ready && ids_ready && cutting_state_.perching_enabled &&
                             cutting_state_.has_locked_pivot && std::isfinite(locked_pivot_error) &&
                             locked_pivot_error <= cutting_state_.locked_pivot_match_tolerance;

      tf::Vector3 contact_pos_world;
      tf::Vector3 saw_com_world;
      if(cutting_state_.valid)
        {
          contact_pos_world = sitePosition(mujoco_data_, cutting_state_.contact_site_id);
          saw_com_world = bodyPosition(mujoco_data_, cutting_state_.saw_body_id);
          cutting_state_.valid = finiteVector(contact_pos_world) && finiteVector(saw_com_world);
        }

      if(cutting_state_.valid)
        {
          const tf::Vector3 radius_world = contact_pos_world - cutting_state_.locked_pivot_world;
          const tf::Vector3 axis_world = cutting_state_.pivot_axis_world;
          const tf::Vector3 ref_perp = cutting_state_.reference_radius_world - axis_world * cutting_state_.reference_radius_world.dot(axis_world);
          const tf::Vector3 radius_perp = radius_world - axis_world * radius_world.dot(axis_world);

          if(cutting_state_.cutting_active && (!cutting_state_.cutting_active_prev || !cutting_state_.has_reference_radius))
            {
              cutting_state_.reference_radius_world = radius_world;
              cutting_state_.has_reference_radius = true;
              cutting_state_.last_signed_travel = 0.0;
            }

          if(cutting_state_.has_reference_radius)
            {
              tf::Vector3 ref_norm = ref_perp;
              tf::Vector3 radius_norm = radius_perp;
              const bool ref_ok = normalizeVector(ref_norm);
              const bool radius_ok = normalizeVector(radius_norm);
              const double radius_length = radius_perp.length();
              const bool geometry_ok = ref_ok && radius_ok && std::isfinite(radius_length) && radius_length > 1e-6;

              double signed_travel = 0.0;
              double tangential_velocity = 0.0;
              if(geometry_ok)
                {
                  const double atan_numerator = axis_world.dot(ref_norm.cross(radius_norm));
                  const double atan_denominator = ref_norm.dot(radius_norm);
                  arc_angle = std::atan2(atan_numerator, atan_denominator);
                  signed_travel = cutting_state_.cutting_direction_sign * cutting_state_.reference_radius_world.length() * arc_angle;
                  const double dt = period.toSec();
                  if(std::isfinite(dt) && dt > 0.0)
                    tangential_velocity = (signed_travel - cutting_state_.last_signed_travel) / dt;

                  aerial_robot_simulation::MujocoCuttingForceModel::Input input;
                  input.enabled = true;
                  input.cutting_active = cutting_state_.cutting_active;
                  input.valid_geometry = geometry_ok;
                  input.dt = period.toSec();
                  input.signed_travel = signed_travel;
                  input.tangential_velocity = tangential_velocity;
                  model_output = cutting_state_.force_model.update(input);
                  cutting_state_.contact = model_output.contact;
                  cutting_state_.completed = model_output.completed;
                  cutting_state_.last_signed_travel = signed_travel;

                  if(model_output.applied_force > 0.0)
                    {
                      tf::Vector3 tangent_plus = axis_world.cross(radius_perp);
                      if(normalizeVector(tangent_plus))
                        {
                          applied_force_world = -model_output.applied_force * cutting_state_.cutting_direction_sign * tangent_plus;
                          torque_cog_world = (contact_pos_world - saw_com_world).cross(applied_force_world);
                          torque_pivot_world = (contact_pos_world - cutting_state_.locked_pivot_world).cross(applied_force_world);

                          for(int axis = 0; axis < 3; ++axis)
                            mujoco_data_->xfrc_applied[6 * cutting_state_.saw_body_id + axis] = applied_force_world[axis];
                          for(int axis = 0; axis < 3; ++axis)
                            mujoco_data_->xfrc_applied[6 * cutting_state_.saw_body_id + 3 + axis] = torque_cog_world[axis];
                        }
                    }
                }
              else
                {
                  cutting_state_.force_model.reset();
                }
            }
          else
            {
              cutting_state_.force_model.reset();
            }
        }
      else
        {
          cutting_state_.contact = false;
          cutting_state_.completed = false;
          cutting_state_.has_reference_radius = false;
          cutting_state_.last_signed_travel = 0.0;
          cutting_state_.force_model.reset();
        }

      cutting_state_.cutting_active_prev = cutting_state_.cutting_active;
      publishCuttingTopics(time, applied_force_world, torque_cog_world, torque_pivot_world, model_output, arc_angle, locked_pivot_error);
    }

  DefaultRobotHWSim::write(time, period);
}

void AerialRobotHWSim::perchingEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  cutting_state_.perching_enabled = msg->data;
  if(!cutting_state_.perching_enabled)
    resetCuttingState();
}

void AerialRobotHWSim::cuttingActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{
  cutting_state_.cutting_active = msg->data;
}

void AerialRobotHWSim::lockedPivotCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  cutting_state_.locked_pivot_world.setValue(msg->point.x, msg->point.y, msg->point.z);
  cutting_state_.has_locked_pivot = finiteVector(cutting_state_.locked_pivot_world);
}

void AerialRobotHWSim::cuttingResetCallback(const std_msgs::Empty::ConstPtr&)
{
  cutting_state_.reset_requested = true;
}

bool AerialRobotHWSim::initCutting(ros::NodeHandle model_nh)
{
  ros::NodeHandle cutting_nh(model_nh, "simulation/cutting");
  cutting_nh.param("enabled", cutting_state_.enabled, false);
  cutting_state_.model_ready = false;

  if(!cutting_state_.enabled)
    return true;

  try
    {
      XmlRpc::XmlRpcValue configured_pivot_world;
      XmlRpc::XmlRpcValue pivot_axis_world;
      cutting_nh.getParam("configured_pivot_world", configured_pivot_world);
      cutting_nh.getParam("pivot_axis_world", pivot_axis_world);
      cutting_state_.configured_pivot_world = xmlRpcVector3(configured_pivot_world, "configured_pivot_world");
      cutting_state_.pivot_axis_world = xmlRpcVector3(pivot_axis_world, "pivot_axis_world");
      if(!normalizeVector(cutting_state_.pivot_axis_world))
        throw std::runtime_error("pivot_axis_world must be non-zero");

      cutting_nh.param("cutting_direction_sign", cutting_state_.cutting_direction_sign, 1.0);
      cutting_nh.param("locked_pivot_match_tolerance", cutting_state_.locked_pivot_match_tolerance, 0.002);

      std::string base_body_name;
      std::string saw_body_name;
      std::string contact_site_name;
      std::string constraint_1_name;
      std::string constraint_2_name;
      std::string perching_enable_topic;
      std::string cutting_active_topic;
      std::string locked_pivot_topic;
      std::string reset_topic;
      std::string selected_profile;
      cutting_nh.param("base_body_name", base_body_name, std::string("base_link"));
      cutting_nh.param("saw_body_name", saw_body_name, std::string("saw"));
      cutting_nh.param("contact_site_name", contact_site_name, std::string("cutting_contact_site"));
      cutting_nh.param("constraint_1_name", constraint_1_name, std::string("perching_hinge_connect_1"));
      cutting_nh.param("constraint_2_name", constraint_2_name, std::string("perching_hinge_connect_2"));
      cutting_nh.param("perching_enable_topic", perching_enable_topic, std::string("perching/enable"));
      cutting_nh.param("cutting_active_topic", cutting_active_topic, std::string("perching/cutting_active"));
      cutting_nh.param("locked_pivot_topic", locked_pivot_topic, std::string("perching/locked_pivot"));
      cutting_nh.param("reset_topic", reset_topic, std::string("simulation/cutting/reset"));
      cutting_nh.param("selected_profile", selected_profile, std::string("bag_1"));

      cutting_state_.base_body_id = mj_name2id(mujoco_model_, mjOBJ_BODY, base_body_name.c_str());
      cutting_state_.saw_body_id = mj_name2id(mujoco_model_, mjOBJ_BODY, saw_body_name.c_str());
      cutting_state_.contact_site_id = mj_name2id(mujoco_model_, mjOBJ_SITE, contact_site_name.c_str());
      cutting_state_.constraint_1_id = mj_name2id(mujoco_model_, mjOBJ_EQUALITY, constraint_1_name.c_str());
      cutting_state_.constraint_2_id = mj_name2id(mujoco_model_, mjOBJ_EQUALITY, constraint_2_name.c_str());

      XmlRpc::XmlRpcValue profiles_xml;
      if(!cutting_nh.getParam("profiles", profiles_xml) || !profiles_xml.hasMember(selected_profile))
        throw std::runtime_error("selected_profile not found: " + selected_profile);
      const XmlRpc::XmlRpcValue& profile_xml = profiles_xml[selected_profile];
      aerial_robot_simulation::MujocoCuttingForceModel::Profile profile;
      profile.preload_force = static_cast<double>(profile_xml["preload_force"]);
      profile.maximum_force = static_cast<double>(profile_xml["maximum_force"]);
      profile.ripple_amplitude = static_cast<double>(profile_xml["ripple_amplitude"]);
      profile.ripple_wavelength = static_cast<double>(profile_xml["ripple_wavelength"]);
      for(int i = 0; i < profile_xml["layers"].size(); ++i)
        {
          aerial_robot_simulation::MujocoCuttingForceModel::Layer layer;
          layer.thickness = static_cast<double>(profile_xml["layers"][i]["thickness"]);
          layer.stiffness = static_cast<double>(profile_xml["layers"][i]["stiffness"]);
          layer.damping = static_cast<double>(profile_xml["layers"][i]["damping"]);
          profile.layers.push_back(layer);
        }

      aerial_robot_simulation::MujocoCuttingForceModel::Config config;
      cutting_nh.param("free_travel", config.free_travel, 0.0);
      cutting_nh.param("force_ramp_time", config.force_ramp_time, 0.0);
      cutting_nh.param("force_filter_time_constant", config.force_filter_time_constant, 0.0);
      cutting_nh.param("maximum_control_dt", config.maximum_control_dt, 0.1);
      cutting_nh.param("noise_enabled", config.noise_enabled, false);
      cutting_nh.param("noise_stddev", config.noise_stddev, 0.0);
      int noise_seed = 1;
      cutting_nh.param("noise_seed", noise_seed, 1);
      config.noise_seed = static_cast<unsigned int>(std::max(noise_seed, 0));

      std::string error;
      if(!cutting_state_.force_model.configure(profile, config, &error))
        throw std::runtime_error(error);

      perching_enable_sub_ = model_nh.subscribe(perching_enable_topic, 1, &AerialRobotHWSim::perchingEnableCallback, this);
      cutting_active_sub_ = model_nh.subscribe(cutting_active_topic, 1, &AerialRobotHWSim::cuttingActiveCallback, this);
      locked_pivot_sub_ = model_nh.subscribe(locked_pivot_topic, 1, &AerialRobotHWSim::lockedPivotCallback, this);
      cutting_reset_sub_ = model_nh.subscribe(reset_topic, 1, &AerialRobotHWSim::cuttingResetCallback, this);

      cutting_model_ready_pub_ = model_nh.advertise<std_msgs::Bool>("simulation/cutting/model_ready", 1, true);
      cutting_valid_pub_ = model_nh.advertise<std_msgs::Bool>("simulation/cutting/valid", 1, true);
      cutting_wrench_cog_pub_ = model_nh.advertise<geometry_msgs::WrenchStamped>("simulation/cutting/wrench_cog_ground_truth", 1);
      cutting_wrench_pivot_pub_ = model_nh.advertise<geometry_msgs::WrenchStamped>("simulation/cutting/wrench_pivot_ground_truth", 1);
      cutting_force_magnitude_pub_ = model_nh.advertise<std_msgs::Float64>("simulation/cutting/force_magnitude", 1);
      cutting_pivot_torque_y_pub_ = model_nh.advertise<std_msgs::Float64>("simulation/cutting/pivot_torque_y_ground_truth", 1);
      cutting_penetration_pub_ = model_nh.advertise<std_msgs::Float64>("simulation/cutting/penetration", 1);
      cutting_penetration_velocity_pub_ = model_nh.advertise<std_msgs::Float64>("simulation/cutting/penetration_velocity", 1);
      cutting_arc_angle_pub_ = model_nh.advertise<std_msgs::Float64>("simulation/cutting/arc_angle", 1);
      cutting_layer_index_pub_ = model_nh.advertise<std_msgs::Int32>("simulation/cutting/layer_index", 1);
      cutting_contact_pub_ = model_nh.advertise<std_msgs::Bool>("simulation/cutting/contact", 1);
      cutting_completed_pub_ = model_nh.advertise<std_msgs::Bool>("simulation/cutting/completed", 1);
      cutting_locked_pivot_error_pub_ = model_nh.advertise<std_msgs::Float64>("simulation/cutting/locked_pivot_error", 1);

      cutting_state_.model_ready = cutting_state_.base_body_id >= 0 && cutting_state_.saw_body_id >= 0 &&
                                   cutting_state_.contact_site_id >= 0 && cutting_state_.constraint_1_id >= 0 &&
                                   cutting_state_.constraint_2_id >= 0 && cutting_state_.force_model.isConfigured();
      publishBool(cutting_model_ready_pub_, ros::Time::now(), cutting_state_.model_ready);
      return true;
    }
  catch(const std::exception& ex)
    {
      ROS_ERROR_STREAM("[mujoco cutting] initialization failed: " << ex.what());
      cutting_state_.model_ready = false;
      if(cutting_model_ready_pub_)
        publishBool(cutting_model_ready_pub_, ros::Time::now(), false);
      return false;
    }
}

void AerialRobotHWSim::resetCuttingState()
{
  cutting_state_.contact = false;
  cutting_state_.completed = false;
  cutting_state_.has_reference_radius = false;
  cutting_state_.last_signed_travel = 0.0;
  cutting_state_.force_model.reset();
}

void AerialRobotHWSim::publishCuttingTopics(const ros::Time& time,
                                            const tf::Vector3& force_world,
                                            const tf::Vector3& torque_cog_world,
                                            const tf::Vector3& torque_pivot_world,
                                            const aerial_robot_simulation::MujocoCuttingForceModel::Output& model_output,
                                            const double arc_angle,
                                            const double locked_pivot_error)
{
  publishBool(cutting_model_ready_pub_, time, cutting_state_.model_ready);
  publishBool(cutting_valid_pub_, time, cutting_state_.valid);
  publishFloat(cutting_force_magnitude_pub_, model_output.applied_force);
  publishFloat(cutting_pivot_torque_y_pub_, torque_pivot_world.y());
  publishFloat(cutting_penetration_pub_, model_output.penetration);
  publishFloat(cutting_penetration_velocity_pub_, model_output.penetration_velocity);
  publishFloat(cutting_arc_angle_pub_, arc_angle);
  publishInt(cutting_layer_index_pub_, model_output.layer_index);
  publishBool(cutting_contact_pub_, time, model_output.contact);
  publishBool(cutting_completed_pub_, time, model_output.completed);
  publishFloat(cutting_locked_pivot_error_pub_, std::isfinite(locked_pivot_error) ? locked_pivot_error : -1.0);

  geometry_msgs::WrenchStamped cog_msg;
  cog_msg.header.stamp = time;
  cog_msg.header.frame_id = "world";
  cog_msg.wrench.force.x = force_world.x();
  cog_msg.wrench.force.y = force_world.y();
  cog_msg.wrench.force.z = force_world.z();
  cog_msg.wrench.torque.x = torque_cog_world.x();
  cog_msg.wrench.torque.y = torque_cog_world.y();
  cog_msg.wrench.torque.z = torque_cog_world.z();
  cutting_wrench_cog_pub_.publish(cog_msg);

  geometry_msgs::WrenchStamped pivot_msg = cog_msg;
  pivot_msg.wrench.torque.x = torque_pivot_world.x();
  pivot_msg.wrench.torque.y = torque_pivot_world.y();
  pivot_msg.wrench.torque.z = torque_pivot_world.z();
  cutting_wrench_pivot_pub_.publish(pivot_msg);
}

}  // namespace mujoco_ros_control

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::AerialRobotHWSim, mujoco_ros_control::RobotHWSim)
