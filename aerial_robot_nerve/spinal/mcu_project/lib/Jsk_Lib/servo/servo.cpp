/**
******************************************************************************
* File Name          : servo.cpp
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
 ------------------------------------------------------------------*/


#include "servo.h"
#include <limits.h>

namespace {
#if SUPPORT_RTOS
// Enclose the complete request/reply or calibration transaction, including
// state publication. The core flight task never waits for this mutex.
class ServoTransaction {
 public:
  explicit ServoTransaction(osMutexId mutex): mutex_(mutex), locked_(
      __get_IPSR() == 0 && mutex && osRecursiveMutexWait(mutex, osWaitForever) == osOK) {}
  ~ServoTransaction() { if (locked_) osRecursiveMutexRelease(mutex_); }
  explicit operator bool() const { return locked_; }
 private:
  osMutexId mutex_;
  bool locked_;
};
#define SERVO_TRANSACTION() ServoTransaction transaction(transaction_mutex_); if (!transaction) return
#else
#define SERVO_TRANSACTION()
#endif
}

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
  
#if SUPPORT_RTOS
  osMutexDef(directServoTransaction);
  transaction_mutex_ = osRecursiveMutexCreate(osMutex(directServoTransaction));
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
#if DYNAMIXEL
  reported_read_failures_ = servo_handler_.getReadFailures();
#endif

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
  SERVO_TRANSACTION();
  applyPendingControl();
  servo_handler_.update();
  publishTargetStates(HAL_GetTick());
  sendData(true);
}

void DirectServo::sendData(bool flag_send_asap)
{
  SERVO_TRANSACTION();
  uint32_t now_time = HAL_GetTick();

  bool ready = true;
  unsigned int enabled = 0;
  for (unsigned i = 0; i < servo_handler_.getServoNum(); ++i) {
    const ServoData& s = servo_handler_.getServo()[i];
    if (!s.send_data_flag_) continue;
    ++enabled;
#if DYNAMIXEL
    ready = ready && s.position_valid_ && s.position_updated_ && now_time - s.position_tick_ <= 100;
#endif
    ready = ready && s.present_position_ >= INT16_MIN && s.present_position_ <= INT16_MAX;
  }
  if (enabled && ready && ((flag_send_asap && servo_handler_.getROSCommFlag()) ||
                          now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)) {
    servo_state_msg_.servos_length = 0;
    servo_state_msg_.stamp = nh_->now();
    for (unsigned i = 0; i < servo_handler_.getServoNum(); ++i) {
      ServoData& s = servo_handler_.getServo()[i];
      if (!s.send_data_flag_) continue;
      spinal::ServoState& state = servo_state_msg_.servos[servo_state_msg_.servos_length++];
      state.index = i;
      state.angle = static_cast<int16_t>(s.present_position_);
      state.temp = s.present_temp_;
      state.load = s.present_current_;
      state.error = s.hardware_error_status_;
      s.position_updated_ = false;
    }
    servo_state_pub_.publish(&servo_state_msg_);
    servo_last_pub_time_ = now_time;
    servo_handler_.setROSCommFlag(false);
  }
#if DYNAMIXEL
  if (now_time - feedback_warning_tick_ >= 1000) {
    const uint32_t failures = servo_handler_.getReadFailures();
    if (failures != reported_read_failures_ || (enabled && now_time - servo_last_pub_time_ > 100)) {
      nh_->logwarn("Servo UART: invalid/missing replies or stale position; check servo bus.");
      reported_read_failures_ = failures;
    }
    feedback_warning_tick_ = now_time;
  }
#endif

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
#if SUPPORT_RTOS
  taskENTER_CRITICAL();
#endif
  for (const auto& servo : servo_map) {
    if (servo.first >= MAX_SERVO_NUM) continue;
    PendingControl& pending = pending_control_[servo.first];
    pending.torque = servo.second != 0;
    pending.torque_pending = true;
  }
#if SUPPORT_RTOS
  taskEXIT_CRITICAL();
#endif
}

void DirectServo::setGoalAngle(const std::map<uint8_t, float>& servo_map, uint8_t value_type)
{
  if (value_type != ValueType::BIT && value_type != ValueType::RADIAN) return;
#if SUPPORT_RTOS
  taskENTER_CRITICAL();
#endif
  for (const auto& servo : servo_map) {
    if (servo.first >= MAX_SERVO_NUM) continue;
    PendingControl& pending = pending_control_[servo.first];
    pending.goal = servo.second;
    pending.type = value_type;
    pending.goal_pending = true;
    pending.torque = true;
    pending.torque_pending = true;
  }
#if SUPPORT_RTOS
  taskEXIT_CRITICAL();
#endif
}

void DirectServo::applyPendingControl()
{
  // Snapshot each slot briefly; never hold a critical section across UART/ROS IO.
  bool has_targets = false;
  for (unsigned i = 0; i < servo_handler_.getServoNum(); ++i) {
#if SUPPORT_RTOS
    taskENTER_CRITICAL();
#endif
    const PendingControl pending = pending_control_[i];
    pending_control_[i].goal_pending = false;
    pending_control_[i].torque_pending = false;
#if SUPPORT_RTOS
    taskEXIT_CRITICAL();
#endif
    ServoData& s = servo_handler_.getServo()[i];
    bool valid_goal = true;
    if (pending.goal_pending) {
      int32_t goal = 0;
      valid_goal = std::isfinite(pending.goal);
      if (valid_goal && pending.type == ValueType::RADIAN) {
        valid_goal = radianToClampedGoalPosition(i, pending.goal, goal);
      } else if (valid_goal) {
        valid_goal = pending.goal >= INT32_MIN && static_cast<double>(pending.goal) <= INT32_MAX;
        if (valid_goal) goal = clampGoalPosition(i, static_cast<int32_t>(pending.goal));
      }
      if (valid_goal) {
        s.setGoalPosition(goal);
        if (!has_targets) {
          servo_target_state_msg_.index_length = 0;
          servo_target_state_msg_.angles_length = 0;
          has_targets = true;
        }
        servo_target_indices_[servo_target_state_msg_.index_length++] = i;
        servo_target_angles_[servo_target_state_msg_.angles_length++] = static_cast<int16_t>(goal);
      }
    }
    if (pending.torque_pending && s.torque_enable_ != pending.torque &&
        (valid_goal || !pending.torque)) {
      s.torque_enable_ = pending.torque;
      servo_handler_.setTorque(i);
    }
  }
}

void DirectServo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
{
  SERVO_TRANSACTION();
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
  SERVO_TRANSACTION();
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
  res.success = false;
  SERVO_TRANSACTION();
  // Reject malformed payloads before accessing data or changing configuration.
  const unsigned required = req.command == spinal::SetDirectServoConfig::Request::SET_DYNAMIXEL_TTL_RS485_MIXED ? 1 :
      req.command == spinal::SetDirectServoConfig::Request::SET_SERVO_PID_GAIN ? 4 :
      req.command == spinal::SetDirectServoConfig::Request::SET_SERVO_RESOLUTION_RATIO ? 3 : 2;
  if (req.data_length != required) {
    nh_->logerror("Invalid direct servo configuration payload.");
    return;
  }
  //TODO: using boardConfigCallback in spine.cpp
  uint8_t command = req.command;

  /* special case : data[0] is flag value */
  if(command == spinal::SetDirectServoConfig::Request::SET_DYNAMIXEL_TTL_RS485_MIXED)
    {
#if DYNAMIXEL
      if (req.data[0] != 0 && req.data[0] != 1) return;
      const uint16_t previous = servo_handler_.getTTLRS485Mixed();
      servo_handler_.setTTLRS485Mixed(req.data[0]);
      res.success = FlashMemory::erase() && FlashMemory::write();
      if (!res.success) {
        servo_handler_.setTTLRS485Mixed(previous);
        nh_->logerror("Failed to persist servo bus mode.");
      }
#endif
      return;
    }

  if (req.data[0] < 0 || static_cast<unsigned>(req.data[0]) >= servo_handler_.getServoNum()) return;
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
#if DYNAMIXEL
        res.success = servo_handler_.calibrateHomingOffset(servo_index, calib_value);
        char message[128];
        snprintf(message, sizeof(message), "Servo calibration id=%u target=%ld offset=%ld verified=%u",
                 s.id_, static_cast<long>(calib_value), static_cast<long>(s.homing_offset_), res.success);
        if (res.success) nh_->loginfo(message);
        else nh_->logerror(message);
#else
        servo_handler_.setHomingOffset(servo_index);
        res.success = true;
#endif
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
        res.success = FlashMemory::erase() && FlashMemory::write();
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
      res.success = FlashMemory::erase() && FlashMemory::write();
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_SEND_DATA_FLAG:
    {
      s.send_data_flag_ = req.data[1];
      res.success = FlashMemory::erase() && FlashMemory::write();
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
#if DYNAMIXEL && defined(SPINAL)
        if (req.data[1] != 0) {
          nh_->logerror("External servo encoder is unsupported on Spinal.");
          return;
        }
#endif
        s.external_encoder_flag_ = req.data[1];
        s.first_get_pos_flag_ = true;
        if(!s.external_encoder_flag_)
          { // if use the servo internal encoder, we directly output the encoder value without scaling by resolution_ratio.
            s.servo_resolution_ = 1;
            s.joint_resolution_ = 1;
            s.resolution_ratio_ = 1;
          }
        res.success = FlashMemory::erase() && FlashMemory::write();
      }else{
        nh_->logerror("Cannot set ex encoder falg during torque on state.");
        res.success = false;
      }
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_RESOLUTION_RATIO:
    {
      if(!s.torque_enable_){
        if (req.data[1] <= 0 || req.data[1] >= 65535 || req.data[2] <= 0 || req.data[2] >= 65535) return;
        s.joint_resolution_ = req.data[1];
        s.servo_resolution_ = req.data[2];
        s.hardware_error_status_ &= ((1 << RESOLUTION_RATIO_ERROR) - 1); // 0b00111111: reset

        s.resolution_ratio_ = static_cast<float>(s.servo_resolution_) / s.joint_resolution_;
        s.first_get_pos_flag_ = true;
        res.success = FlashMemory::erase() && FlashMemory::write();
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
  SERVO_TRANSACTION();
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
  SERVO_TRANSACTION();
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
