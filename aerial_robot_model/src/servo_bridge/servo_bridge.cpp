// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <aerial_robot_model/servo_bridge.h>
#include <sstream>

namespace
{
constexpr int32_t kDefaultRawLowerLimit = 0;
constexpr int32_t kDefaultRawUpperLimit = 4095;
}

std::string ServoBridge::servoErrorToString(uint8_t error) const
{
  std::vector<std::string> error_list;
  if(error & 0b10000000) error_list.push_back("Encoder Connection Error");
  if(error & 0b01000000) error_list.push_back("Resolution Ratio Error");
  if(error & 0b00100000) error_list.push_back("Overload Error");
  if(error & 0b00010000) error_list.push_back("Electrical Shock Error");
  if(error & 0b00001000) error_list.push_back("Motor Encoder Error");
  if(error & 0b00000100) error_list.push_back("Overheating Error");
  if(error & 0b00000010) error_list.push_back("Pulley Skip Error");
  if(error & 0b00000001) error_list.push_back("Input Voltage Error");

  if(error_list.empty()) return "No Error";

  std::ostringstream oss;
  for(size_t i = 0; i < error_list.size(); ++i)
    {
      if(i > 0) oss << ", ";
      oss << error_list[i];
    }
  return oss.str();
}

void ServoBridge::reportServoErrorState(int servo_index, uint8_t error, const std::string& servo_group_name)
{
  const ServoErrorKey key(servo_group_name, servo_index);
  auto prev_it = servo_error_states_.find(key);
  if(prev_it != servo_error_states_.end() && prev_it->second == error) return;

  servo_error_states_[key] = error;

  std::string servo_name = "unknown";
  auto group_it = servos_handler_.find(servo_group_name);
  if(group_it != servos_handler_.end())
    {
      auto servo_handler = find_if(group_it->second.begin(), group_it->second.end(),
                                   [&](SingleServoHandlePtr s) {return servo_index == s->getId();});
      if(servo_handler != group_it->second.end()) servo_name = (*servo_handler)->getName();
    }

  if(error == 0)
    {
      if(prev_it != servo_error_states_.end() && prev_it->second != 0)
        ROS_INFO("[servo bridge] servo %d (%s, group: %s) error cleared",
                 servo_index, servo_name.c_str(), servo_group_name.c_str());
      return;
    }

  ROS_WARN("[servo bridge] servo %d (%s, group: %s) error: %s",
           servo_index, servo_name.c_str(), servo_group_name.c_str(), servoErrorToString(error).c_str());
}

ServoBridge::ServoBridge(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh),nhp_(nhp)
{
  nh_.param("/use_sim_time", simulation_mode_, false);
  nhp_.param("use_mujoco", use_mujoco_, false);
  if(use_mujoco_)
    {
      ROS_WARN("use mujoco simulator");
      simulation_mode_ = false;
    }

  if(simulation_mode_)
    {
      if(!ros::service::waitForService(nh_.getNamespace() + std::string("/controller_manager/load_controller")))
        {
          ROS_ERROR("can't find the controller manager: %s", (nh_.getNamespace() + std::string("/controller_manager/load_controller")).c_str());
          return;
        }
    }

  /* get robot URDF model via robot_description*/
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
    ROS_ERROR("Failed to extract urdf model from rosparam");

  /* common param and topics (between servo_bridge and spinal_ros_bridge) */
  ros::NodeHandle nh_controller(nh_, "servo_controller");
  std::string state_sub_topic, pos_pub_topic, torque_pub_topic, servo_enable_pub_topic;
  nh_controller.param("state_sub_topic", state_sub_topic, std::string("servo/states"));
  nh_controller.param("pos_pub_topic", pos_pub_topic, std::string("servo/target_states"));
  nh_controller.param("torque_pub_topic", torque_pub_topic, std::string("servo/target_current"));
  nh_controller.param("servo_enable_pub_topic", servo_enable_pub_topic, std::string("servo/torque_enable"));
  /* common subsriber: get servo states (i.e. joint angles) from real machine (spinal_ros_bridge) */
  servo_states_subs_.insert(make_pair("common", nh_.subscribe<spinal::ServoStates>(state_sub_topic, 10, boost::bind(&ServoBridge::servoStatesCallback, this, _1, "common"))));
  /* common publisher: target servo position to real machine (spinal_ros_bridge) */
  servo_target_pos_pubs_.insert(make_pair("common", nh_.advertise<spinal::ServoControlCmd>(pos_pub_topic, 1)));
  mujoco_control_input_pub_ = nh_.advertise<sensor_msgs::JointState>("mujoco/ctrl_input", 1);
  /* common publisher: target servo torque to real machine (spinal_ros_bridge) */
  servo_target_torque_pubs_.insert(make_pair("common", nh_.advertise<spinal::ServoControlCmd>(torque_pub_topic, 1)));
  /* common publisher: servo on/off flag to real machine (spinal_ros_bridge) */
  servo_enable_pubs_.insert(make_pair("common", nh_.advertise<spinal::ServoTorqueCmd>(servo_enable_pub_topic, 1)));
  /* common publisher: joint profiles */
  joint_profile_pub_ = nh_.advertise<spinal::JointProfiles>("joint_profiles",1);
  /* subscriber: uav info */
  uav_info_sub_ = nh_.subscribe<spinal::UavInfo>("uav_info", 1, &ServoBridge::uavInfoCallback, this);
  servo_target_states_debug_sub_ = nh_.subscribe<spinal::ServoControlCmd>("servo/target_states_debug", 1, &ServoBridge::servoTargetStatesDebugCallback, this);

  /* get additional config for servos from ros parameters */
  XmlRpc::XmlRpcValue all_servos_params;
  nh_.getParam("servo_controller", all_servos_params);
  spinal::JointProfiles joint_profiles_msg;
  for(auto servo_group_params: all_servos_params)
    {
      if (servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        continue;

      std::string group_name = servo_group_params.first;

      /* check whether has feedback state from real machine */
      no_real_state_flags_.insert(make_pair(group_name, false));
      if(servo_group_params.second.hasMember("no_real_state"))
        no_real_state_flags_.at(group_name) = servo_group_params.second["no_real_state"];

      /* pub, sub, and srv */
      /* mandatory subscriber: target servo command (e.g., position, torque) from controller */
      servo_ctrl_subs_.insert(make_pair(group_name, nh_.subscribe<sensor_msgs::JointState>(group_name + string("_ctrl"), 10, boost::bind(&ServoBridge::servoCtrlCallback, this, _1, group_name))));
      /* mandatory subscriber: only target servo troque command from controller */
      servo_torque_ctrl_subs_.insert(make_pair(group_name, nh_.subscribe<sensor_msgs::JointState>(group_name + string("_torque_ctrl"), 10, boost::bind(&ServoBridge::servoTorqueCtrlCallback, this, _1, group_name))));
      /* mandatory service: get torque enalbe/disable flag  */
      servo_enable_srvs_.insert(make_pair(group_name, nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(servo_group_params.first + string("/torque_enable"), boost::bind(&ServoBridge::servoEnableCallback, this, _1, _2, servo_group_params.first))));

      if(servo_group_params.second.hasMember("state_sub_topic"))
        {
          /* option: servo states (i.e. joint angles) from real machine (spinal_ros_bridge) */
          servo_states_subs_.insert(make_pair(group_name, nh_.subscribe<spinal::ServoStates>((string)servo_group_params.second["state_sub_topic"], 10, boost::bind(&ServoBridge::servoStatesCallback, this, _1, servo_group_params.first))));
        }

      if(servo_group_params.second.hasMember("pos_pub_topic"))
        {
          /* option: publish target servo state to real machine (spinal_ros_bridge) */
          servo_target_pos_pubs_.insert(make_pair(group_name, nh_.advertise<spinal::ServoControlCmd>(servo_group_params.second["pos_pub_topic"], 1)));
        }

      if(servo_group_params.second.hasMember("torque_pub_topic"))
        {
          /* option: torque on/off */
          servo_target_torque_pubs_.insert(make_pair(group_name, nh_.advertise<spinal::ServoControlCmd>(servo_group_params.second["torque_pub_topic"], 1)));
        }

      if(servo_group_params.second.hasMember("servo_enable_pub_topic"))
        {
          /* option: torque on/off */
          servo_enable_pubs_.insert(make_pair(group_name, nh_.advertise<spinal::ServoTorqueCmd>((string)servo_group_params.second["servo_enable_pub_topic"], 1)));
        }

      /* servo handler */
      ServoGroupHandler servo_group_handler;
      for(auto servo_params : servo_group_params.second)
        {
          if(servo_params.first.find("controller") != string::npos)
            {
              /* get parameters from urdf file */
              double upper_limit = urdf_model.getJoint(servo_params.second["name"])->limits->upper;
              double lower_limit = urdf_model.getJoint(servo_params.second["name"])->limits->lower;

              /* get parameters from rosparam */
              int servo_id = servo_params.second["id"]; 
              int angle_sgn = servo_params.second.hasMember("angle_sgn")? 
                servo_params.second["angle_sgn"]:servo_group_params.second["angle_sgn"];
              int zero_point_offset = servo_params.second.hasMember("zero_point_offset")?
                servo_params.second["zero_point_offset"]:servo_group_params.second["zero_point_offset"];
              double angle_scale = servo_params.second.hasMember("angle_scale")?
                servo_params.second["angle_scale"]:servo_group_params.second["angle_scale"];
              int32_t raw_lower_limit = kDefaultRawLowerLimit;
              int32_t raw_upper_limit = kDefaultRawUpperLimit;
              const bool has_raw_lower_limit = servo_params.second.hasMember("raw_lower_limit") || servo_group_params.second.hasMember("raw_lower_limit");
              const bool has_raw_upper_limit = servo_params.second.hasMember("raw_upper_limit") || servo_group_params.second.hasMember("raw_upper_limit");
              if(has_raw_lower_limit)
                raw_lower_limit = servo_params.second.hasMember("raw_lower_limit") ?
                  static_cast<int32_t>(servo_params.second["raw_lower_limit"]) :
                  static_cast<int32_t>(servo_group_params.second["raw_lower_limit"]);
              if(has_raw_upper_limit)
                raw_upper_limit = servo_params.second.hasMember("raw_upper_limit") ?
                  static_cast<int32_t>(servo_params.second["raw_upper_limit"]) :
                  static_cast<int32_t>(servo_group_params.second["raw_upper_limit"]);
              if(!(has_raw_lower_limit && has_raw_upper_limit))
                {
                  const std::string servo_name = static_cast<std::string>(servo_params.second["name"]);
                  if(raw_limit_default_warned_servos_.insert(servo_name).second)
                    ROS_WARN("[servo bridge] servo %s uses fallback raw limits [%d, %d]",
                             servo_name.c_str(), kDefaultRawLowerLimit, kDefaultRawUpperLimit);
                }

              double torque_scale = servo_group_params.second.hasMember("torque_scale")?
                servo_group_params.second["torque_scale"]:(servo_params.second.hasMember("torque_scale")?servo_params.second["torque_scale"]: XmlRpc::XmlRpcValue(1.0));

              /* for low pass filtering */
              bool filter_flag = servo_group_params.second.hasMember("filter_flag")?
                servo_group_params.second["filter_flag"]:(servo_params.second.hasMember("filter_flag")?servo_params.second["filter_flag"]: XmlRpc::XmlRpcValue(false));
              double sample_freq = servo_group_params.second.hasMember("sample_freq")?
                servo_group_params.second["sample_freq"]:(servo_params.second.hasMember("sample_freq")?servo_params.second["sample_freq"]: XmlRpc::XmlRpcValue(0.0));
              double cutoff_freq = servo_group_params.second.hasMember("cutoff_freq")?
                servo_group_params.second["cutoff_freq"]:(servo_params.second.hasMember("cutoff_freq")?servo_params.second["cutoff_freq"]: XmlRpc::XmlRpcValue(0.0));

              servo_group_handler.push_back(SingleServoHandlePtr(new SingleServoHandle(servo_params.second["name"], servo_params.second["id"], angle_sgn, zero_point_offset, angle_scale, upper_limit, lower_limit, torque_scale, raw_lower_limit, raw_upper_limit, !no_real_state_flags_.at(group_name), filter_flag, sample_freq, cutoff_freq)));

              /* rosparam and load controller for gazebo */
              if(simulation_mode_)
                {
                  if(!servo_group_params.second.hasMember("simulation") &&
                     !servo_params.second.hasMember("simulation"))
                    {
                      ROS_ERROR("please set gazebo servo parameters for %s, using sub namespace 'simulation:'", string(servo_params.second["name"]).c_str());

                      continue;
                    }

                  vector<string> parameter_names = {"type", "pid", "init_value"};
                  for(auto parameter_name : parameter_names)
                    {
                      if(!servo_params.second.hasMember("simulation") ||
                         (servo_params.second.hasMember("simulation") &&
                          !servo_params.second["simulation"].hasMember(parameter_name)))
                        {
                          if(!servo_group_params.second["simulation"].hasMember(parameter_name))
                            {
                              ROS_ERROR("can not find '%s' gazebo paramter for servo %s", parameter_name.c_str(),  string(servo_params.second["name"]).c_str());
                              return;
                            }
                          nh_.setParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/") + parameter_name, servo_group_params.second["simulation"][parameter_name]);
                        }
                    }
                  nh_.setParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/joint"), servo_params.second["name"]);


                  /* load the controller */
                  ros::ServiceClient controller_loader = nh_.serviceClient<controller_manager_msgs::LoadController>(nh_.getNamespace() + std::string("/controller_manager/load_controller"));
                  controller_manager_msgs::LoadController load_srv;
                  load_srv.request.name = nh_.getNamespace() + string("/servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation");

                  if (controller_loader.call(load_srv))
                    ROS_INFO("load gazebo controller for %s", string(servo_params.second["name"]).c_str());
                  else
                    ROS_ERROR("Failed to call service %s", controller_loader.getService().c_str());

                  /* start the controller */
                  ros::ServiceClient controller_starter = nh_.serviceClient<controller_manager_msgs::SwitchController>(nh_.getNamespace() + std::string("/controller_manager/switch_controller"));
                  controller_manager_msgs::SwitchController switch_srv;
                  switch_srv.request.start_controllers.push_back(load_srv.request.name);
                  switch_srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

                  if (controller_starter.call(switch_srv))
                    ROS_INFO("start servo controller for %s", string(servo_params.second["name"]).c_str());
                  else
                    ROS_ERROR("Failed to call service %s", controller_starter.getService().c_str());

                  /* init the servo command publisher to the controller */
                  servo_target_pos_sim_pubs_[servo_group_params.first].push_back(nh_.advertise<std_msgs::Float64>(load_srv.request.name + string("/command"), 1));
                  // wait for the publisher initialization
                  while(servo_target_pos_sim_pubs_[servo_group_params.first].back().getNumSubscribers() == 0 && ros::ok())
                    ros::Duration(0.1).sleep();

                  /* set the initial angle */
                  std_msgs::Float64 msg;
                  nh_.getParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/init_value"), msg.data);
                  servo_target_pos_sim_pubs_[servo_group_params.first].back().publish(msg);
                }
            }
        }

      servos_handler_.insert(make_pair(servo_group_params.first, servo_group_handler));
    }

std::map<int, std::string> servo_id_to_group;
for(const auto& servo_group : servos_handler_)
  {
    for(const auto& servo : servo_group.second)
      {
        const int servo_id = servo->getId();

        auto prev = servo_id_to_group.find(servo_id);
        if(prev != servo_id_to_group.end())
          {
            ROS_ERROR("[servo bridge] duplicate servo id %d in groups '%s' and '%s'. "
                      "servo/target_states_debug cannot be converted safely.",
                      servo_id,
                      prev->second.c_str(),
                      servo_group.first.c_str());
          }
        else
          {
            servo_id_to_group[servo_id] = servo_group.first;
          }
      }
  }

  if(!simulation_mode_)
  {
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

    servo_states_pub_ = nh_.advertise<sensor_msgs::JointState>("servos/states", 1);
    servo_target_states_pub_ = nh_.advertise<sensor_msgs::JointState>("servos/target_states", 1);

    gimbal_states_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals/states", 1);
    gimbal_target_states_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals/target_states", 1);
  }
}

void ServoBridge::publishGimbalStates(const ros::Time& stamp)
{
  const auto group_it = servos_handler_.find("gimbals");
  if(group_it == servos_handler_.end()) return;

  sensor_msgs::JointState msg;
  msg.header.stamp = stamp;

  for(const auto& servo : group_it->second)
    {
      msg.name.push_back(servo->getName());
      msg.position.push_back(servo->getCurrAngleVal(ValueType::RADIAN));
      msg.effort.push_back(servo->getCurrTorqueVal());
    }

  if(!msg.name.empty()) gimbal_states_pub_.publish(msg);
}

void ServoBridge::publishServoStates(const ros::Time& stamp)
{
  sensor_msgs::JointState msg;
  msg.header.stamp = stamp;

  for(const auto& servo_group : servos_handler_)
    {
      for(const auto& servo : servo_group.second)
        {
          msg.name.push_back(servo->getName());
          msg.position.push_back(servo->getCurrAngleVal(ValueType::RADIAN));
          msg.effort.push_back(servo->getCurrTorqueVal());
        }
    }

  if(!msg.name.empty())
    servo_states_pub_.publish(msg);
}

void ServoBridge::servoStatesCallback(const spinal::ServoStatesConstPtr& state_msg, const string& servo_group_name)
{
  /* independent group servo process without state publish */
  if(servo_group_name != std::string("common"))
    {
      for(auto it: state_msg->servos)
        {
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(),
                                       servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return it.index == s->getId();} );

          if(servo_handler == servos_handler_[servo_group_name].end())
            {
              ROS_ERROR("[servo bridge, servo state callback]: no matching joint handler for servo index %d", it.index);
              return;
            }
          reportServoErrorState(it.index, it.error, servo_group_name);
          (*servo_handler)->setCurrAngleVal((double)it.angle, ValueType::BIT); // angle (position)
          (*servo_handler)->setCurrTorqueVal((double)it.load); // torque (effort)
        }

      publishServoStates(state_msg->stamp);
      if(servo_group_name == "gimbals") publishGimbalStates(state_msg->stamp);

      // finish process
      return;
    }


  /* group of "common", search in all group */
  for(auto it: state_msg->servos)
    {
      for(auto servo_group: servos_handler_)
        {
          // no real state from real machine, skip
          if(no_real_state_flags_.at(servo_group.first))
            {
              ROS_DEBUG_STREAM(servo_group.first << ", no real state, skip");
              continue;
            }

          // has independent servo state process
          if(servo_states_subs_.find(servo_group.first) != servo_states_subs_.end())
            {
              ROS_DEBUG_STREAM(servo_group.first << ", do not process in common group, skip");
              continue;
            }

          auto servo_handler = find_if(servos_handler_[servo_group.first].begin(),
                                       servos_handler_[servo_group.first].end(),
                                       [&](SingleServoHandlePtr s) {return it.index == s->getId();});

          if(servo_handler == servos_handler_[servo_group.first].end())
            {
              // search in next servo group
              continue;
            }

          reportServoErrorState(it.index, it.error, servo_group.first);
          (*servo_handler)->setCurrAngleVal((double)it.angle, ValueType::BIT); // angle (position)
          (*servo_handler)->setCurrTorqueVal((double)it.load); // torque (effort)

          ROS_DEBUG("servo index: %d, find in group %s", it.index, servo_group.first.c_str());

          break; // search next servo state from real machine
        }
    }

  sensor_msgs::JointState servo_states_msg;
  servo_states_msg.header.stamp = state_msg->stamp;

  for(auto servo_group : servos_handler_)
    {
      for(auto servo_handler: servo_group.second)
        {
          servo_states_msg.name.push_back(servo_handler->getName());
          servo_states_msg.position.push_back(servo_handler->getCurrAngleVal(ValueType::RADIAN));
          servo_states_msg.effort.push_back(servo_handler->getCurrTorqueVal());
        }
    }
  joint_states_pub_.publish(servo_states_msg);
  publishServoStates(state_msg->stamp);
  publishGimbalStates(state_msg->stamp);
}

void ServoBridge::servoTargetStatesDebugCallback(const spinal::ServoControlCmdConstPtr& msg)
{
  publishServoAndGimbalTargetStates(msg);
}

void ServoBridge::publishServoAndGimbalTargetStates(const spinal::ServoControlCmdConstPtr& msg)
{
  sensor_msgs::JointState servo_target_msg;
  sensor_msgs::JointState gimbal_target_msg;

  const ros::Time stamp = ros::Time::now();
  servo_target_msg.header.stamp = stamp;
  gimbal_target_msg.header.stamp = stamp;

  const size_t target_num = std::min(msg->index.size(), msg->angles.size());

  for(size_t i = 0; i < target_num; ++i)
    {
      const uint8_t servo_index = msg->index.at(i);
      const double raw_goal = static_cast<double>(msg->angles.at(i));

      bool matched = false;

      for(const auto& servo_group : servos_handler_)
        {
          auto servo_handler =
              find_if(servo_group.second.begin(),
                      servo_group.second.end(),
                      [&](SingleServoHandlePtr s)
                      {
                        return servo_index == s->getId();
                      });

          if(servo_handler == servo_group.second.end())
            continue;

          const double target_rad =
              (*servo_handler)->getAngleScale() *
              (*servo_handler)->getAngleSgn() *
              (raw_goal - (*servo_handler)->getZeroPointOffset());

          servo_target_msg.name.push_back((*servo_handler)->getName());
          servo_target_msg.position.push_back(target_rad);

          if(servo_group.first == "gimbals")
            {
              gimbal_target_msg.name.push_back((*servo_handler)->getName());
              gimbal_target_msg.position.push_back(target_rad);
            }

          matched = true;
          break;
        }

      if(!matched)
        {
          ROS_WARN_THROTTLE(1.0,
                            "[servo bridge] target debug for unknown servo index %d",
                            servo_index);
        }
    }

  if(!servo_target_msg.name.empty())
    servo_target_states_pub_.publish(servo_target_msg);

  if(!gimbal_target_msg.name.empty())
    gimbal_target_states_pub_.publish(gimbal_target_msg);
}

void ServoBridge::servoCtrlCallback(const sensor_msgs::JointStateConstPtr& servo_ctrl_msg, const string& servo_group_name)
{
  spinal::ServoControlCmd target_angle_msg;
  spinal::ServoControlCmd target_torque_msg;
  sensor_msgs::JointState mujoco_control_input_msg;

  if(servo_ctrl_msg->name.size() > 0)
    {
      for(int i = 0; i < servo_ctrl_msg->name.size(); i++)
        {/* servo name is assigned */
          if(servo_ctrl_msg->position.size() !=  servo_ctrl_msg->name.size())
            {
              ROS_ERROR("[servo bridge, servo control control]: the servo position num and name num are different in ros msgs [%d vs %d]",
                        (int)servo_ctrl_msg->position.size(), (int)servo_ctrl_msg->name.size());
              return;
            }

          mujoco_control_input_msg.name.push_back(servo_ctrl_msg->name.at(i));
          mujoco_control_input_msg.position.push_back(servo_ctrl_msg->position.at(i));

          // use servo_name to search the servo_handler
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(), servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return servo_ctrl_msg->name.at(i)  == s->getName();});

          if(servo_handler == servos_handler_[servo_group_name].end())
          {
            ROS_ERROR("[servo bridge, servo control callback]: no matching servo handler for %s", servo_ctrl_msg->name.at(i).c_str());
            return;
          }

          (*servo_handler)->setTargetAngleVal(servo_ctrl_msg->position[i], ValueType::RADIAN);
          target_angle_msg.index.push_back((*servo_handler)->getId());
          target_angle_msg.angles.push_back((*servo_handler)->getTargetAngleVal(ValueType::BIT));

          // process torque command if necessary
          if(servo_ctrl_msg->effort.size() == servo_ctrl_msg->name.size())
            {
              double torque = servo_ctrl_msg->effort.at(i);
              (*servo_handler)->setTargetTorqueVal(torque);
              target_torque_msg.index.push_back((*servo_handler)->getId());
              target_torque_msg.angles.push_back((*servo_handler)->getTargetTorqueVal(ValueType::BIT));
            }

          if(simulation_mode_)
            {
              std_msgs::Float64 msg;
              msg.data = servo_ctrl_msg->position[i];
              servo_target_pos_sim_pubs_[servo_group_name].at(distance(servos_handler_[servo_group_name].begin(), servo_handler)).publish(msg);
            }
        }
    }
  else
    { /* for fast tranmission: no searching process, in the predefine order */

      if(servo_ctrl_msg->position.size() != servos_handler_[servo_group_name].size())
        {
          ROS_ERROR("[servo bridge, servo ctrl control]: the joint num from rosparam %d is not equal with ros msgs %d",
                    (int)servos_handler_[servo_group_name].size(), (int)servo_ctrl_msg->position.size());
          return;
        }

      for(int i = 0; i < servo_ctrl_msg->position.size(); i++)
        {
          /*  use the kinematics order (e.g. joint1 ~ joint N, gimbal_roll -> gimbal_pitch) */
          SingleServoHandlePtr servo_handler = servos_handler_[servo_group_name].at(i);
          servo_handler->setTargetAngleVal(servo_ctrl_msg->position[i], ValueType::RADIAN);
          target_angle_msg.index.push_back(servo_handler->getId());
          target_angle_msg.angles.push_back(servo_handler->getTargetAngleVal(ValueType::BIT));

          // process torque command if necessary
          if(servo_ctrl_msg->effort.size() == servo_ctrl_msg->position.size())
            {
              double torque = servo_ctrl_msg->effort.at(i);
              servo_handler->setTargetTorqueVal(torque);
              target_torque_msg.index.push_back(servo_handler->getId());
              target_torque_msg.angles.push_back(servo_handler->getTargetTorqueVal(ValueType::BIT));
            }

          mujoco_control_input_msg.name.push_back(servo_handler->getName());
          mujoco_control_input_msg.position.push_back(servo_ctrl_msg->position.at(i));

          if(simulation_mode_)
            {
              std_msgs::Float64 msg;
              msg.data = servo_ctrl_msg->position[i];
              servo_target_pos_sim_pubs_[servo_group_name].at(i).publish(msg);
            }
        }
    }

  mujoco_control_input_pub_.publish(mujoco_control_input_msg);

  if (servo_target_pos_pubs_.find(servo_group_name) != servo_target_pos_pubs_.end())
    servo_target_pos_pubs_[servo_group_name].publish(target_angle_msg);
  else
    servo_target_pos_pubs_["common"].publish(target_angle_msg);


  // process torque command if necessary
  if (target_torque_msg.index.size() == 0) return;
  if (servo_target_torque_pubs_.find(servo_group_name) != servo_target_torque_pubs_.end())
    servo_target_torque_pubs_[servo_group_name].publish(target_torque_msg);
  else
    servo_target_torque_pubs_["common"].publish(target_torque_msg);
}

void ServoBridge::servoTorqueCtrlCallback(const sensor_msgs::JointStateConstPtr& servo_ctrl_msg, const string& servo_group_name)
{
  spinal::ServoControlCmd target_torque_msg;

  if(servo_ctrl_msg->name.size() > 0)
    {
      for(int i = 0; i < servo_ctrl_msg->name.size(); i++)
        {/* servo name is assigned */

          if(servo_ctrl_msg->effort.size() !=  servo_ctrl_msg->name.size())
            {
              ROS_ERROR("[servo bridge, servo torque control]: the servo effort (torque) num and name num are different in ros msgs [%d vs %d]",
                        (int)servo_ctrl_msg->effort.size(), (int)servo_ctrl_msg->name.size());
              return;
            }

          // use servo_name to search the servo_handler
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(), servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return servo_ctrl_msg->name.at(i)  == s->getName();});

          if(servo_handler == servos_handler_[servo_group_name].end())
          {
            ROS_ERROR("[servo bridge, servo control callback]: no matching servo handler for %s", servo_ctrl_msg->name.at(i).c_str());
            return;
          }

          double torque = servo_ctrl_msg->effort.at(i);
          (*servo_handler)->setTargetTorqueVal(torque);
          target_torque_msg.index.push_back((*servo_handler)->getId());
          target_torque_msg.angles.push_back((*servo_handler)->getTargetTorqueVal(ValueType::BIT));
        }
    }
  else
    { /* for fast tranmission: no searching process, in the predefine order */

      if(servo_ctrl_msg->position.size() != servos_handler_[servo_group_name].size())
        {
          ROS_ERROR("[servo bridge, servo control control]: the joint num from rosparam %d is not equal with ros msgs %d",
                    (int)servos_handler_[servo_group_name].size(), (int)servo_ctrl_msg->position.size());
          return;
        }

      for(int i = 0; i < servo_ctrl_msg->position.size(); i++)
        {
          /*  use the kinematics order (e.g. joint1 ~ joint N, gimbal_roll -> gimbal_pitch) */
          SingleServoHandlePtr servo_handler = servos_handler_[servo_group_name].at(i);
          double torque = servo_ctrl_msg->effort.at(i);

          servo_handler->setTargetTorqueVal(torque);
          target_torque_msg.index.push_back(servo_handler->getId());
          target_torque_msg.angles.push_back(servo_handler->getTargetTorqueVal(ValueType::BIT));
        }
    }

  if (servo_target_torque_pubs_.find(servo_group_name) != servo_target_torque_pubs_.end())
    servo_target_torque_pubs_[servo_group_name].publish(target_torque_msg);
  else
    servo_target_torque_pubs_["common"].publish(target_torque_msg);
}

bool ServoBridge::servoEnableCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& servo_group_name)
{
  spinal::ServoTorqueCmd torque_msg;
  for(auto servo_handler: servos_handler_[servo_group_name])
    {
      torque_msg.index.push_back(servo_handler->getId());
      torque_msg.torque_enable.push_back(req.data);
    }

  if (servo_enable_pubs_.find(servo_group_name) != servo_enable_pubs_.end())
    servo_enable_pubs_[servo_group_name].publish(torque_msg);
  else
    servo_enable_pubs_["common"].publish(torque_msg);

  return true;
}

void ServoBridge::uavInfoCallback(const spinal::UavInfoConstPtr& uav_msg)
{
  /* Send servo profiles to Spinal*/
  spinal::JointProfiles joint_profiles_msg;
  for(auto servo_group : servos_handler_){
    for(auto servo : servo_group.second){
      spinal::JointProfile joint_profile;
      if(servo_group.first == "joints"){
        joint_profile.type = spinal::JointProfile::JOINT;
      }
      else if(servo_group.first == "gimbals"){
        joint_profile.type = spinal::JointProfile::GIMBAL;
      }
      else{
        ROS_ERROR("Invalid servo type. Please define 'joints' or 'gimbals'.");
        continue;
      }
      joint_profile.servo_id = servo->getId();
      joint_profile.angle_sgn = servo->getAngleSgn();
      joint_profile.angle_scale = servo->getAngleScale();
      joint_profile.zero_point_offset = servo->getZeroPointOffset();
      joint_profile.lower_limit = servo->getLowerLimit();
      joint_profile.upper_limit = servo->getUpperLimit();
      joint_profile.raw_lower_limit = servo->getRawLowerLimit();
      joint_profile.raw_upper_limit = servo->getRawUpperLimit();
      joint_profiles_msg.joints.push_back(joint_profile);
    }
  }
  joint_profile_pub_.publish(joint_profiles_msg);
}
