/**
******************************************************************************
* File Name          : servo.cpp
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
 ------------------------------------------------------------------*/


#include "servo.h"

#define  SERVO_PUB_INTERVAL 20 // 50Hz
#define SERVO_TORQUE_PUB_INTERVAL  1000 // 1Hz
#define SERVO_TARGET_PUB_INTERVAL 20 // 50Hz
#define DIRECT_SERVO_DEFAULT_RAW_LOWER_LIMIT 0
#define DIRECT_SERVO_DEFAULT_RAW_UPPER_LIMIT 4095
#define DIRECT_SERVO_DEFAULT_LOWER_LIMIT_RAD -1.50f
#define DIRECT_SERVO_DEFAULT_UPPER_LIMIT_RAD 1.50f

void DirectServo::init(UART_HandleTypeDef* huart,  ros::NodeHandle* nh, osMutexId* mutex = NULL) //TODO: support encoder
{
  /*setup pin configuration*/
#if !STM32H7_V2
#ifdef STM32H7
  HAL_UART_DeInit(huart);
  huart->Init.BaudRate = 1000000;
  HAL_UART_Init(huart);
#endif
#endif
  
  nh_ = nh;
  nh_->subscribe(servo_ctrl_sub_);
  nh_->subscribe(servo_torque_ctrl_sub_);
  nh_->subscribe(joint_profiles_sub_);
  nh_->advertise(servo_target_state_pub_);
  nh_->advertise(servo_state_pub_);
  nh_->advertise(servo_torque_state_pub_);
  nh_->advertiseService(servo_config_srv_);
  nh_->advertiseService(board_info_srv_);

  //temp
  servo_handler_.init(huart, mutex);

  unsigned int actual_servo_num = servo_handler_.getServoNum();

  servo_state_msg_.servos_length = actual_servo_num;
  servo_state_msg_.servos = new spinal::ServoState[actual_servo_num];
  servo_torque_state_msg_.torque_enable_length = actual_servo_num;
  servo_torque_state_msg_.torque_enable = new uint8_t[actual_servo_num];
  servo_target_state_msg_.index_length = 0;
  servo_target_state_msg_.index = servo_target_indices_;
  servo_target_state_msg_.angles_length = 0;
  servo_target_state_msg_.angles = servo_target_angles_;

  for (unsigned int i = 0; i < MAX_SERVO_NUM; i++) {
    joint_profiles_[i].servo_id = i;
    joint_profiles_[i].type = spinal::JointProfile::GIMBAL;
    joint_profiles_[i].angle_sgn = 1;
    joint_profiles_[i].angle_scale = 0.001534f;
    joint_profiles_[i].zero_point_offset = 2047;
    joint_profiles_[i].lower_limit = DIRECT_SERVO_DEFAULT_LOWER_LIMIT_RAD;
    joint_profiles_[i].upper_limit = DIRECT_SERVO_DEFAULT_UPPER_LIMIT_RAD;
    joint_profiles_[i].raw_lower_limit = DIRECT_SERVO_DEFAULT_RAW_LOWER_LIMIT;
    joint_profiles_[i].raw_upper_limit = DIRECT_SERVO_DEFAULT_RAW_UPPER_LIMIT;
    joint_profile_received_[i] = false;
    servo_target_indices_[i] = i;
    servo_target_angles_[i] = 2047;
  }

  servo_target_last_pub_time_ = 0;
  servo_last_pub_time_ = 0;
  servo_torque_last_pub_time_ = 0;

  board_info_res_.boards_length = 1;
  board_info_res_.boards = new spinal::BoardInfo[1];
  board_info_res_.boards[0].servos_length = servo_handler_.getServoNum();
  board_info_res_.boards[0].servos = new spinal::ServoInfo[servo_handler_.getServoNum()];
}

int32_t DirectServo::clampGoalPosition(uint8_t index, int32_t goal_pos)
{
  if (index >= MAX_SERVO_NUM) return goal_pos;

  const JointProf& joint_prof = joint_profiles_[index];
  const int32_t raw_lower =
      joint_prof.raw_lower_limit < joint_prof.raw_upper_limit ?
      joint_prof.raw_lower_limit : joint_prof.raw_upper_limit;
  const int32_t raw_upper =
      joint_prof.raw_lower_limit < joint_prof.raw_upper_limit ?
      joint_prof.raw_upper_limit : joint_prof.raw_lower_limit;

  if(joint_prof.angle_scale > 0.0f && joint_prof.angle_sgn != 0) {
    int32_t min_mech_goal =
      static_cast<int32_t>(joint_prof.lower_limit * joint_prof.angle_sgn / joint_prof.angle_scale +
                           joint_prof.zero_point_offset);
    int32_t max_mech_goal =
      static_cast<int32_t>(joint_prof.upper_limit * joint_prof.angle_sgn / joint_prof.angle_scale +
                           joint_prof.zero_point_offset);
    if(min_mech_goal > max_mech_goal) {
      int32_t tmp = min_mech_goal;
      min_mech_goal = max_mech_goal;
      max_mech_goal = tmp;
    }

    if(goal_pos < min_mech_goal) goal_pos = min_mech_goal;
    else if(goal_pos > max_mech_goal) goal_pos = max_mech_goal;
  }

  if(goal_pos < raw_lower) return raw_lower;
  if(goal_pos > raw_upper) return raw_upper;
  return goal_pos;
}

bool DirectServo::radianToClampedGoalPosition(uint8_t index, float angle, int32_t& goal_pos)
{
  if(index >= MAX_SERVO_NUM) return false;
  if(!joint_profile_received_[index]) {
    nh_->logerror("Servo profile has not been received.");
    return false;
  }

  const JointProf& joint_prof = joint_profiles_[index];
  if(joint_prof.angle_scale <= 0.0f || joint_prof.angle_sgn == 0) {
    nh_->logerror("Invalid Servo Joint Profile!");
    return false;
  }

  float lower_limit = joint_prof.lower_limit;
  float upper_limit = joint_prof.upper_limit;
  if(lower_limit > upper_limit) {
    float tmp = lower_limit;
    lower_limit = upper_limit;
    upper_limit = tmp;
  }

  if(angle > upper_limit) angle = upper_limit;
  else if(angle < lower_limit) angle = lower_limit;

  goal_pos = static_cast<int32_t>(angle * joint_prof.angle_sgn /
                                  joint_prof.angle_scale +
                                  joint_prof.zero_point_offset);
  goal_pos = clampGoalPosition(index, goal_pos);
  return true;
}

void DirectServo::publishTargetStates(uint32_t now_time)
{
  if(now_time - servo_target_last_pub_time_ < SERVO_TARGET_PUB_INTERVAL) return;
  servo_target_state_pub_.publish(&servo_target_state_msg_);
  servo_target_last_pub_time_ = now_time;
}

void DirectServo::update()
{
  servo_handler_.update();
  publishTargetStates(HAL_GetTick());
  sendData(true);
}

void DirectServo::sendData(bool flag_send_asap)
{
  uint32_t now_time = HAL_GetTick();

  if (flag_send_asap && servo_handler_.getROSCommFlag() == true)  // This setting will ignore the setting of SERVO_PUB_INTERVAL and pub the information once the measurement is updated.
  {
	  servo_state_msg_.stamp = nh_->now();
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        const ServoData& s = servo_handler_.getServo()[i];
        if (s.send_data_flag_ != 0) {
          spinal::ServoState servo;
          servo.index = i;
          servo.angle = s.present_position_;
          servo.temp = s.present_temp_;
          servo.load = s.present_current_;
          servo.error = s.hardware_error_status_;
          servo_state_msg_.servos[i] = servo;
        }
      }
      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;

      servo_handler_.setROSCommFlag(false);
  }
  else
  {
  if( now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
    {
	  servo_state_msg_.stamp = nh_->now();
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        const ServoData& s = servo_handler_.getServo()[i];
        if (s.send_data_flag_ != 0) {
          spinal::ServoState servo;
          servo.index = i;
          servo.angle = s.present_position_;
          servo.temp = s.present_temp_;
          servo.load = s.present_current_;
          servo.error = s.hardware_error_status_;
          servo_state_msg_.servos[i] = servo;
        }
      }
      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;
    }
  }

  if( now_time - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL)
    {
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        const ServoData& s = servo_handler_.getServo()[i];
        if (s.send_data_flag_ != 0) {
          servo_torque_state_msg_.torque_enable[i] = s.torque_enable_;
        }
      }
      servo_torque_state_pub_.publish(&servo_torque_state_msg_);
      servo_torque_last_pub_time_= now_time;
    }  
}

void DirectServo::torqueEnable(const std::map<uint8_t, float>& servo_map)
{
  for (auto servo : servo_map)
    {
      uint8_t index = servo.first;
      if(index >= servo_handler_.getServoNum())
        {
          nh_->logerror("Invalid Servo ID!");
          return;
        }
      ServoData& s = servo_handler_.getServo()[index];
      if(servo.second && !s.torque_enable_){
        s.torque_enable_ = true;
        servo_handler_.setTorque(index);
      }
      else if(!servo.second && s.torque_enable_){
        s.torque_enable_ = false;
        servo_handler_.setTorque(index);
      }     
    }
}

void DirectServo::setGoalAngle(const std::map<uint8_t, float>& servo_map, uint8_t value_type)
{
  servo_target_state_msg_.index_length = 0;
  servo_target_state_msg_.angles_length = 0;

  for (auto servo : servo_map)
    {
      uint8_t index = servo.first;
      if(index >= servo_handler_.getServoNum())
        {
          nh_->logerror("Invalid Servo ID!");
          return;
        }

      int32_t goal_pos;
      if(value_type == ValueType::BIT){
        goal_pos = clampGoalPosition(index, static_cast<int32_t>(servo.second));
      }else if(value_type == ValueType::RADIAN){
        if(!radianToClampedGoalPosition(index, servo.second, goal_pos)) continue;
      }else{
        nh_->logerror("Invalid Servo Value Type!");
        return;
      }

      ServoData& s = servo_handler_.getServo()[index];
      s.setGoalPosition(goal_pos);
      if(servo_target_state_msg_.index_length < MAX_SERVO_NUM) {
        servo_target_indices_[servo_target_state_msg_.index_length] = index;
        servo_target_angles_[servo_target_state_msg_.angles_length] = static_cast<int16_t>(goal_pos);
        servo_target_state_msg_.index_length++;
        servo_target_state_msg_.angles_length++;
      }
      if (! s.torque_enable_) {
        s.torque_enable_ = true;
        servo_handler_.setTorque(index);
      }
      
    }
}

void DirectServo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
{
  if (control_msg.index_length != control_msg.angles_length) return;
  servo_target_state_msg_.index_length = 0;
  servo_target_state_msg_.angles_length = 0;

  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    uint8_t index = control_msg.index[i];
    if(index >= servo_handler_.getServoNum())
      {
        nh_->logerror("Invalid Servo ID!");
        return;
    }
    ServoData& s = servo_handler_.getServo()[index];
    int32_t goal_pos = clampGoalPosition(index, static_cast<int32_t>(control_msg.angles[i]));
    s.setGoalPosition(goal_pos);
    if(servo_target_state_msg_.index_length < MAX_SERVO_NUM) {
      servo_target_indices_[servo_target_state_msg_.index_length] = index;
      servo_target_angles_[servo_target_state_msg_.angles_length] = static_cast<int16_t>(goal_pos);
      servo_target_state_msg_.index_length++;
      servo_target_state_msg_.angles_length++;
    }
    if (! s.torque_enable_) {
      s.torque_enable_ = true;
      servo_handler_.setTorque(index);
    }
  }
}

void DirectServo::servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
{
  if (control_msg.index_length != control_msg.torque_enable_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    uint8_t index = control_msg.index[i];
    if(index >= servo_handler_.getServoNum())
      {
        nh_->logerror("Invalid Servo ID!");
        return;
      }
    ServoData& s = servo_handler_.getServo()[index];
    s.torque_enable_ = (control_msg.torque_enable[i] != 0) ? true : false;
    servo_handler_.setTorqueFromPresetnPos(index);

  }
}

void DirectServo::servoConfigCallback(const spinal::SetDirectServoConfig::Request& req, spinal::SetDirectServoConfig::Response& res)
{
  //TODO: using boardConfigCallback in spine.cpp
  uint8_t command = req.command;

  /* special case : data[0] is flag value */
  if(command == spinal::SetDirectServoConfig::Request::SET_DYNAMIXEL_TTL_RS485_MIXED)
    {
      // servo_handler_.setTTLRS485Mixed(req.data[0]);
      FlashMemory::erase();
      FlashMemory::write();
      res.success = true;
      return;
    }

  uint8_t servo_index = req.data[0];
  if(servo_index >= servo_handler_.getServoNum())
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }
  ServoData& s = servo_handler_.getServo()[servo_index];
  switch (command) {
  case spinal::SetDirectServoConfig::Request::SET_SERVO_HOMING_OFFSET:
    {
      if(!s.torque_enable_){
        int32_t calib_value = req.data[1];
        s.calib_value_ = calib_value;
        servo_handler_.setHomingOffset(servo_index);
        res.success = true;
      }else{
        nh_->logerror("Cannot set homing offset during torque on state.");
        res.success = false;
      }
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_PID_GAIN:
    {
      if(!s.torque_enable_){
        s.p_gain_ = req.data[1];
        s.i_gain_ = req.data[2];
        s.d_gain_ = req.data[3];
        servo_handler_.setPositionGains(servo_index);
        FlashMemory::erase();
        FlashMemory::write();
        res.success = true;
      }else{
        nh_->logerror("Cannot set PID gains during torque on state.");
        res.success = false;
      }
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_PROFILE_VEL:
    {
      s.profile_velocity_ = req.data[1];
      servo_handler_.setProfileVelocity(servo_index);
      FlashMemory::erase();
      FlashMemory::write();
      res.success = true;
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_SEND_DATA_FLAG:
    {
      s.send_data_flag_ = req.data[1];
      FlashMemory::erase();
      FlashMemory::write();
      res.success = true;
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_CURRENT_LIMIT:
    {
      s.current_limit_ = req.data[1];
      servo_handler_.setCurrentLimit(servo_index);
      res.success = true;
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_EXTERNAL_ENCODER_FLAG:
    {
      if(!s.torque_enable_){
        s.external_encoder_flag_ = req.data[1];
        s.first_get_pos_flag_ = true;
        if(!s.external_encoder_flag_)
          { // if use the servo internal encoder, we directly output the encoder value without scaling by resolution_ratio.
            s.servo_resolution_ = 1;
            s.joint_resolution_ = 1;
            s.resolution_ratio_ = 1;
          }
        FlashMemory::erase();
        FlashMemory::write();
        res.success = true;
      }else{
        nh_->logerror("Cannot set ex encoder falg during torque on state.");
        res.success = false;
      }
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_RESOLUTION_RATIO:
    {
      if(!s.torque_enable_){
        s.joint_resolution_ = req.data[1];
        s.servo_resolution_ = req.data[2];
        s.hardware_error_status_ &= ((1 << RESOLUTION_RATIO_ERROR) - 1); // 0b00111111: reset

        if(s.servo_resolution_ == 65535 || s.joint_resolution_ == 65535){
          s.hardware_error_status_ |= (1 << RESOLUTION_RATIO_ERROR);  // 0b01000000;
          s.resolution_ratio_ = 1;
        }
        else{
          s.resolution_ratio_ = (float)s.servo_resolution_ / (float)s.joint_resolution_;
          s.first_get_pos_flag_ = true;
          FlashMemory::erase();
          FlashMemory::write();
        }
        res.success = true;
      }else{
        nh_->logerror("Cannot set resolution rate during torque on state.");
        res.success = false;
      }
      
      break;
    }
  default:
    break;
  }
  // res.success = true;
}

void DirectServo::boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res)
{
  //TODO: Bad implementation. This features should not be located in servo interface.
  spinal::BoardInfo& board = board_info_res_.boards[0];
  board.imu_send_data_flag = 1;
#if DYNAMIXEL
  board.dynamixel_ttl_rs485_mixed = servo_handler_.getTTLRS485Mixed();
#endif
  board.slave_id = 0;
  for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
    const ServoData& s = servo_handler_.getServo()[i];
    board.servos[i].id = s.id_;
    board.servos[i].p_gain = s.p_gain_;
    board.servos[i].i_gain = s.i_gain_;
    board.servos[i].d_gain = s.d_gain_;
    board.servos[i].profile_velocity = s.profile_velocity_;
    board.servos[i].current_limit = s.current_limit_;
    board.servos[i].send_data_flag = s.send_data_flag_;
    board.servos[i].external_encoder_flag = s.external_encoder_flag_;
    board.servos[i].joint_resolution = s.joint_resolution_;
    board.servos[i].servo_resolution = s.servo_resolution_;
  }
  res = board_info_res_;
}

void DirectServo::jointProfilesCallback(const spinal::JointProfiles& joint_prof_msg)
{
  for(unsigned int i = 0; i  < joint_prof_msg.joints_length; i++){
    uint8_t servo_id = joint_prof_msg.joints[i].servo_id;
    if(servo_id >= MAX_SERVO_NUM) continue;
    joint_profiles_[servo_id].servo_id = servo_id;
    joint_profiles_[servo_id].type = joint_prof_msg.joints[i].type;
    joint_profiles_[servo_id].angle_sgn = joint_prof_msg.joints[i].angle_sgn;
    joint_profiles_[servo_id].angle_scale = joint_prof_msg.joints[i].angle_scale;
    joint_profiles_[servo_id].zero_point_offset = joint_prof_msg.joints[i].zero_point_offset;
    joint_profiles_[servo_id].lower_limit = joint_prof_msg.joints[i].lower_limit;
    joint_profiles_[servo_id].upper_limit = joint_prof_msg.joints[i].upper_limit;
    joint_profiles_[servo_id].raw_lower_limit = joint_prof_msg.joints[i].raw_lower_limit;
    joint_profiles_[servo_id].raw_upper_limit = joint_prof_msg.joints[i].raw_upper_limit;
    joint_profile_received_[servo_id] = true;
  }
}
