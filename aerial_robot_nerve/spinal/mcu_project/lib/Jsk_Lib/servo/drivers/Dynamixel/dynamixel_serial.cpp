#include "dynamixel_serial.h"
// #include "flashmemory/flashmemory.h"

namespace
{

#if STM32H7_V2
  uint8_t rx_buf_[RX_BUFFER_SIZE] __attribute__((section(".ServoRxBufferSection")));
#else
#ifdef STM32H7
  uint8_t rx_buf_[RX_BUFFER_SIZE] __attribute__((section(".GpsRxBufferSection")));
#else
  uint8_t rx_buf_[RX_BUFFER_SIZE];
#endif
#endif
}
void DynamixelSerial::init(UART_HandleTypeDef* huart, osMutexId* mutex)
{
	huart_ = huart;
  mutex_ = mutex;
	servo_num_ = 0;
	set_pos_tick_ = 0;
	get_pos_tick_ = 0;
	get_load_tick_ = 0;
	get_temp_tick_ = 0;
	get_move_tick_ = 0;
	get_error_tick_ = 0;

  direct_ttl_mode_ = false;

        /* rx */
  rd_ptr_ = 0;
  memset(rx_buf_, 0, sizeof(rx_buf_));

	std::fill(servo_.begin(), servo_.end(), ServoData(255));

        // scan to decide the comm (UART/ Half-duplex) mode
        pinReconfig();

	//initialize servo motors
  cmdReboot(DX_BROADCAST_ID);
	HAL_Delay(3000);

	setStatusReturnLevel();
	//Successfully detected servo's led will be turned on 1 seconds

	for (unsigned int i = 0; i < servo_num_; i++) {
		servo_[i].led_ = true;
	}
	cmdSyncWriteLed();

	HAL_Delay(1000);
	for (unsigned int i = 0; i < servo_num_; i++) {
		servo_[i].led_ = false;
	}
	cmdSyncWriteLed();

	for (int i = 0; i < MAX_SERVO_NUM; i++) {
		FlashMemory::addValue(&(servo_[i].p_gain_), 2);
		FlashMemory::addValue(&(servo_[i].i_gain_), 2);
		FlashMemory::addValue(&(servo_[i].d_gain_), 2);
		FlashMemory::addValue(&(servo_[i].profile_velocity_), 2);
		FlashMemory::addValue(&(servo_[i].send_data_flag_), 2);
                FlashMemory::addValue(&(servo_[i].external_encoder_flag_), 2);
                FlashMemory::addValue(&(servo_[i].joint_resolution_), 2);
                FlashMemory::addValue(&(servo_[i].servo_resolution_), 2);
                FlashMemory::addValue(&(servo_[i].joint_offset_), 4);
	}
	FlashMemory::addValue(&(ttl_rs485_mixed_), 2);

	FlashMemory::read();

	cmdSyncWritePositionGains();
	cmdSyncWriteProfileVelocity();

	getHomingOffset(); // This operation always fails after the servo motor is ignited first
	getHomingOffset(); // So this line is necessary
	getCurrentLimit();
	getPositionGains();
	getProfileVelocity();


        //initialize encoder: only can support one encoder
#ifndef SPINAL
        encoder_handler_.init(hi2c); // Magnetic Encoder
#endif
	for (int i = 0; i < MAX_SERVO_NUM; i++) {
          servo_[i].hardware_error_status_ = 0;
          if(servo_[i].servo_resolution_ == 65535 || servo_[i].joint_resolution_ == 65535){
#ifndef SPINAL
            if(servo_[i].external_encoder_flag_)
              servo_[i].hardware_error_status_ = 1 << RESOLUTION_RATIO_ERROR;  // 0b01000000;
#endif
            servo_[i].resolution_ratio_ = 1;
          }
          else{
            servo_[i].resolution_ratio_ = (float)servo_[i].servo_resolution_ / (float)servo_[i].joint_resolution_;
#ifndef SPINAL
            if(servo_[i].external_encoder_flag_) encoder_handler_.setOffset(servo_[i].joint_offset_);
#endif
          }
	}

        /*set angle scale values*/
        for (int i = 0; i < MAX_SERVO_NUM; i++){
          servo_[i].zero_point_offset_ = 2047;
          servo_[i].angle_scale_ = 3.1416 / 2047;
        }
}

void DynamixelSerial::pinReconfig()
{
  // scan uart mode
  while(HAL_UART_DeInit(huart_) != HAL_OK);
  /*Change baud rate*/
  huart_->Init.BaudRate = 1000000;
  huart_->Init.WordLength = UART_WORDLENGTH_8B;
  huart_->Init.Parity = UART_PARITY_NONE;
  huart_->Init.Mode = UART_MODE_TX_RX;
  while(HAL_UART_Init(huart_) != HAL_OK);

  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
  HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);

  HAL_Delay(500);

  ping();

  // find more than one servo
  if (servo_num_ > 0) return;


  direct_ttl_mode_ = true;

  // scan with TTL mode
  while(HAL_UART_DeInit(huart_) != HAL_OK);
  /*Initialize as halfduplex mode*/
  while(HAL_HalfDuplex_Init(huart_) != HAL_OK);

  // change the pull mode for gpio
  GPIO_InitTypeDef GPIO_InitStruct = {0};
#if STM32H7_V2
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#else
#ifdef STM32H7
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#endif
#endif

  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
  HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);

  HAL_Delay(500);

  ping();

  // find more than one servo
  if (servo_num_ > 0) return;

  direct_ttl_mode_ = false;

  // change back to the no-push-pull mode for gpio
#if STM32H7_V2
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#else
#ifdef STM32H7
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#endif
#endif
}

void DynamixelSerial::ping()
{
	cmdPing(DX_BROADCAST_ID);
	for (int i = 0; i < PING_TRIAL_NUM; ++i) {
		readStatusPacket(INST_PING);
	}
}


void DynamixelSerial::reboot(uint8_t servo_index)
{
	cmdReboot(servo_[servo_index].id_);
}

void DynamixelSerial::setTorque(uint8_t servo_index)
{
  // Share the worker's queue lock, including while a mode change discards old polling work.
  if (mutex_ != NULL && osMutexWait(*mutex_, osWaitForever) != osOK) return;
  instruction_buffer_.push(std::make_pair(INST_SET_TORQUE, servo_index));
  if (mutex_ != NULL) osMutexRelease(*mutex_);
}

void DynamixelSerial::setTorqueFromPresetnPos(uint8_t servo_index)
{
  if (mutex_ != NULL && osMutexWait(*mutex_, osWaitForever) != osOK) return;
  ServoData& s = servo_[servo_index];
  if(s.torque_enable_) s.goal_position_ = s.present_position_;
  instruction_buffer_.push(std::make_pair(INST_SET_TORQUE, servo_index));
  if (mutex_ != NULL) osMutexRelease(*mutex_);
}

void DynamixelSerial::setRoundOffset(uint8_t servo_index, int32_t ref_value)
{
  ServoData& s = servo_[servo_index];
  if (s.torque_enable_) return; // cannot process if torque is enable

  // workaround to update the internal offset for pulley type in the case that has round gap
  int32_t diff = ref_value - s.present_position_;
  if (diff > 2047) s.internal_offset_ += 4096;
  if (diff < -2047) s.internal_offset_ -= 4096;
}

void DynamixelSerial::setHomingOffset(uint8_t servo_index)
{
  if(servo_[servo_index].external_encoder_flag_){
	servo_[servo_index].joint_offset_ = servo_[servo_index].calib_value_ + servo_[servo_index].joint_offset_ - servo_[servo_index].present_position_;
#ifndef SPINAL
	servo_[servo_index].joint_offset_ = servo_[servo_index].calib_value_; // debug
	encoder_handler_.setOffset(servo_[servo_index].joint_offset_);
#endif
	servo_[servo_index].first_get_pos_flag_ = true;
	FlashMemory::erase();
	FlashMemory::write();
  }
  else{
    if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

    // reset homing offset and internal offset
    servo_[servo_index].homing_offset_ = 0;
    servo_[servo_index].internal_offset_ = 0;
    cmdWriteHomingOffset(servo_index);
    cmdReadHomingOffset(servo_index);
    readStatusPacket(INST_GET_HOMING_OFFSET);

    // read updated position
    cmdReadPresentPosition(servo_index);
    readStatusPacket(INST_GET_PRESENT_POS);

    // set new homing offset
    servo_[servo_index].updateHomingOffset();
    cmdWriteHomingOffset(servo_index);
    cmdReadHomingOffset(servo_index);
    readStatusPacket(INST_GET_HOMING_OFFSET);

    // read updated position
    cmdReadPresentPosition(servo_index);
    readStatusPacket(INST_GET_PRESENT_POS);

    if (mutex_ != NULL) osMutexRelease(*mutex_);
  }
}

void DynamixelSerial::setPositionGains(uint8_t servo_index)
{
  if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

  cmdWritePositionGains(servo_index);
  getPositionGains();

  if (mutex_ != NULL) osMutexRelease(*mutex_);
}

void DynamixelSerial::setProfileVelocity(uint8_t servo_index)
{
  if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

  cmdWriteProfileVelocity(servo_index);
  getProfileVelocity();

  if (mutex_ != NULL) osMutexRelease(*mutex_);
}

void DynamixelSerial::setCurrentLimit(uint8_t servo_index)
{
  if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

  cmdWriteCurrentLimit(servo_index);
  getCurrentLimit();

  if (mutex_ != NULL) osMutexRelease(*mutex_);
}

ServoData& DynamixelSerial::getOneServo(uint8_t id)
{
  for(int i = 0; i < servo_num_; i++)
    {
      if(servo_[i].id_ == id) return servo_[i];
    }
  ServoData non_servo;
  return non_servo;
}

uint8_t DynamixelSerial::getServoIndex(uint8_t id)
{
  for(int i = 0; i < servo_num_; i++)
    {
      if(servo_[i].id_ == id) return i;
    }
  return 0;
}

bool DynamixelSerial::setTTLRS485Mixed(uint16_t flag)
{
  if (flag > 1) return false;
  if (mutex_ != NULL && osMutexWait(*mutex_, osWaitForever) != osOK) return false;
  if (ttl_rs485_mixed_ != flag)
  {
    ttl_rs485_mixed_ = flag;
    mode_reset_pending_ = true;
  }
  if (mutex_ != NULL) osMutexRelease(*mutex_);
  return true;
}

void DynamixelSerial::resetPolling()
{
  // Discard work scheduled for the previous mode, preserving explicit control writes.
  const uint16_t queued = instruction_buffer_.length();
  std::pair<uint8_t, uint8_t> instruction;
  for (uint16_t i = 0; i < queued; ++i)
  {
    if (!instruction_buffer_.pop(instruction)) break;
    switch (instruction.first)
    {
    case INST_SET_TORQUE:
    case INST_SET_HOMING_OFFSET:
    case INST_SET_POSITION_GAINS:
    case INST_SET_CURRENT_LIMIT:
    case INST_SET_PROFILE_VELOCITY:
      instruction_buffer_.push(instruction);
      break;
    default:
      break;
    }
  }
  memset(queued_periodic_, 0, sizeof(queued_periodic_));
  read_status_packet_flag_ = false;
  fast_read_pending_ = false;
  requested_count_ = response_index_ = 0;
  requested_size_ = 0;
  instruction_last_ = std::make_pair(255, 255);
  set_pos_tick_ = get_pos_tick_ = get_load_tick_ = 0;
  get_temp_tick_ = get_move_tick_ = get_error_tick_ = 0;
  schedule_initialized_ = false;
  for (auto& s : servo_) s.feedback_fresh_ = false;
  setROSCommFlag(false);
  discardRx();
  rx_recovering_ = true;
  rx_quiet_tick_ = HAL_GetTick();
  mode_reset_pending_ = false;
}

void DynamixelSerial::queuePeriodic(uint8_t instruction, uint8_t index)
{
  const uint16_t bit = uint16_t(1) << index;
  if (queued_periodic_[instruction] & bit) return;
  if (instruction_buffer_.push(std::make_pair(instruction, index)))
    queued_periodic_[instruction] |= bit;
}

bool DynamixelSerial::finishPendingRead()
{
  if (!read_status_packet_flag_) return true;
  read_status_packet_flag_ = false;
  bool success = requested_count_ > 0;
  if (fast_read_pending_)
    success = readFastSyncStatusPacket() == 0;
  else
    for (unsigned int i = 0; i < requested_count_; ++i)
      if (readStatusPacket(instruction_last_.first) != 0) { success = false; break; }
  if (success && (instruction_last_.first == INST_GET_PRESENT_POS ||
                  instruction_last_.first == INST_GET_CUR_VEL_POS))
    setROSCommFlag(true);
  if (!success)
  {
    for (auto& s : servo_) s.feedback_fresh_ = false;
    setROSCommFlag(false);
    discardRx();
    rx_recovering_ = true;
    rx_quiet_tick_ = HAL_GetTick();
  }
  return success;
}

void DynamixelSerial::update()
{
  /* skip if no servo is detected */
  if (servo_num_ == 0) return;

  // Serialize scheduling, mode changes and the complete UART transaction with callbacks.
  if (mutex_ != NULL && osMutexWait(*mutex_, osWaitForever) != osOK) return;
  if (mode_reset_pending_) resetPolling();
  if (rx_recovering_)
  {
    const uint32_t wr = (RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % RX_BUFFER_SIZE;
    if (wr != rd_ptr_) rx_quiet_tick_ = HAL_GetTick();
    rd_ptr_ = wr;
    // After a failed/truncated reply, require two quiet worker ticks before reusing the bus.
    if (HAL_GetTick() - rx_quiet_tick_ < 2)
    {
      if (mutex_ != NULL) osMutexRelease(*mutex_);
      return;
    }
    rx_recovering_ = false;
  }
  uint32_t current_time = HAL_GetTick();
#if STM32H7_V2
  const uint32_t set_pos_interval = ttl_rs485_mixed_ != 0 ? MIXED_SET_POS_DU : SET_POS_DU;
  const uint32_t get_pos_interval = ttl_rs485_mixed_ != 0 ? MIXED_GET_POS_DU : GET_POS_DU;
  if (!schedule_initialized_)
  {
    set_pos_tick_ = current_time + SET_POS_OFFSET;
    get_pos_tick_ = current_time + (ttl_rs485_mixed_ != 0 ? MIXED_GET_POS_OFFSET : GET_POS_OFFSET);
    schedule_initialized_ = true;
  }
#endif

  /* send position command to servo */
#if STM32H7_V2
  if ((int32_t)(current_time - set_pos_tick_) >= 0) {
    // Skip missed slots instead of accumulating duplicate commands after a long operation.
    set_pos_tick_ += ((current_time - set_pos_tick_) / set_pos_interval + 1) * set_pos_interval;
#else
  if(current_time >= set_pos_tick_ + SET_POS_DU && SET_POS_DU > 0) {
    if (set_pos_tick_ == 0) set_pos_tick_ = current_time + SET_POS_OFFSET; // init
    else set_pos_tick_ = current_time;
#endif

    queuePeriodic(INST_SET_GOAL_POS, 0);
  }

  /* read servo position(angle) */
#if STM32H7_V2
  const uint8_t position_instruction = INST_GET_CUR_VEL_POS;
#else
  const uint8_t position_instruction = INST_GET_PRESENT_POS;
#endif
#if STM32H7_V2
  if ((int32_t)(current_time - get_pos_tick_) >= 0) {
    // Skip missed slots instead of accumulating duplicate commands after a long operation.
    get_pos_tick_ += ((current_time - get_pos_tick_) / get_pos_interval + 1) * get_pos_interval;
#else
  if(current_time >= get_pos_tick_ + GET_POS_DU && GET_POS_DU > 0) {
    if (get_pos_tick_ == 0) get_pos_tick_ = current_time + GET_POS_OFFSET; // init
    else get_pos_tick_ = current_time;
#endif

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        queuePeriodic(position_instruction, i);
      }
    } else {
      queuePeriodic(position_instruction, 0);
    }
  }

#if !STM32H7_V2 // V2 current is included in the contiguous feedback read.
  /* read servo load */

  if(current_time >= get_load_tick_ + GET_LOAD_DU && GET_LOAD_DU > 0) {
    if (get_load_tick_ == 0) get_load_tick_ = current_time + GET_LOAD_OFFSET; // init
    else get_load_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        queuePeriodic(INST_GET_PRESENT_CURRENT, i);
      }
    } else {
      queuePeriodic(INST_GET_PRESENT_CURRENT, 0);
    }
  }

#endif

  /* read servo temperature */

  if(current_time >= get_temp_tick_ + GET_TEMP_DU && GET_TEMP_DU > 0) {
    if (get_temp_tick_ == 0) get_temp_tick_ = current_time + GET_TEMP_OFFSET;  // init
    else get_temp_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        queuePeriodic(INST_GET_PRESENT_TEMPERATURE, i);
      }
    } else {
      queuePeriodic(INST_GET_PRESENT_TEMPERATURE, 0);
    }
  }

  /* read servo movement */
  if(current_time >= get_move_tick_ + GET_MOVE_DU && GET_MOVE_DU > 0) {
    if (get_move_tick_ == 0) get_move_tick_ = current_time + GET_MOVE_OFFSET; // init
    else get_move_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        queuePeriodic(INST_GET_PRESENT_MOVING, i);
      }
    } else {
      queuePeriodic(INST_GET_PRESENT_MOVING, 0);
    }
  }

  /* read hardware error status */
  if(current_time >= get_error_tick_ + GET_HARDWARE_ERROR_STATUS_DU && GET_HARDWARE_ERROR_STATUS_DU > 0) {
    if (get_error_tick_ == 0) get_error_tick_ = current_time + GET_HARDWARE_ERROR_STATUS_OFFSET; // init
    else get_error_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        queuePeriodic(INST_GET_HARDWARE_ERROR_STATUS, i);
      }
    } else {
      queuePeriodic(INST_GET_HARDWARE_ERROR_STATUS, 0);
    }
  }


  /* process the instruction from the instruction buffer */
  std::pair<uint8_t, uint8_t> instruction;
  while(instruction_buffer_.pop(instruction))
    {
      uint8_t servo_index = instruction.second;
      if (instruction.first < 17)
        queued_periodic_[instruction.first] &= ~(uint16_t(1) << servo_index);

      /* set command */
      switch (instruction.first) {
      case INST_SET_GOAL_POS: /* send angle command to servo */
        cmdSyncWriteGoalPosition();
        break;
      case INST_SET_TORQUE: /* send torque enable flag */
        cmdWriteTorqueEnable(servo_index);
        break;
      case INST_SET_HOMING_OFFSET:
        cmdWriteHomingOffset(servo_index);
        break;
      case INST_SET_POSITION_GAINS:
        cmdWritePositionGains(servo_index);
        break;
      case INST_SET_CURRENT_LIMIT:
        cmdWriteCurrentLimit(servo_index);
        break;
      case INST_SET_PROFILE_VELOCITY:
        cmdWriteProfileVelocity(servo_index);
        break;
      default:
        break;
      }

      /* get command */
      if (ttl_rs485_mixed_ != 0) {
        switch (instruction.first) {
        case INST_GET_CUR_VEL_POS:
          if(!servo_[servo_index].send_data_flag_ && !servo_[servo_index].first_get_pos_flag_) break;
          cmdReadPresentCurVelPos(servo_index);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_POS: /* read servo position(angle) */
          if(!servo_[servo_index].send_data_flag_ && !servo_[servo_index].first_get_pos_flag_) break;
          cmdReadPresentPosition(servo_index);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_CURRENT: /* read servo load */
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadPresentCurrent(servo_index);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_TEMPERATURE: /* read servo temp */
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadPresentTemperature(servo_index);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_MOVING: /* read servo movement */
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadMoving(servo_index);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_HARDWARE_ERROR_STATUS:
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadHardwareErrorStatus(servo_index);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_HOMING_OFFSET:
          cmdReadHomingOffset(servo_index);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_POSITION_GAINS:
          cmdReadPositionGains(servo_index);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PROFILE_VELOCITY:
          cmdReadProfileVelocity(servo_index);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_CURRENT_LIMIT:
          cmdReadCurrentLimit(servo_index);
          read_status_packet_flag_ = true;
          break;
        default:
          break;
        }
      } else {
        switch (instruction.first) {
        case INST_GET_CUR_VEL_POS:
          cmdSyncReadPresentCurVelPos();
          read_status_packet_flag_ = requested_count_ > 0;
          break;
        case INST_GET_PRESENT_POS: /* read servo position(angle) */
          cmdSyncReadPresentPosition(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_CURRENT: /* read servo load */
          cmdSyncReadPresentCurrent(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_TEMPERATURE: /* read servo temp */
          cmdSyncReadPresentTemperature(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_MOVING: /* read servo movement */
          cmdSyncReadMoving(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_HARDWARE_ERROR_STATUS:
          cmdSyncReadHardwareErrorStatus(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_HOMING_OFFSET:
          cmdSyncReadHomingOffset(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_POSITION_GAINS:
          cmdSyncReadPositionGains(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PROFILE_VELOCITY:
          cmdSyncReadProfileVelocity(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_CURRENT_LIMIT:
          cmdSyncReadCurrentLimit(false);
          read_status_packet_flag_ = true;
          break;
        default:
          break;
        }

      }

      if (read_status_packet_flag_) instruction_last_ = instruction;
      const bool read_sent = read_status_packet_flag_;
      finishPendingRead();
      // Service at most one read transaction per worker iteration. Slow buses coalesce
      // cyclic work instead of overwriting unread replies or growing a command backlog.
      if (read_sent || rx_recovering_) break;
    }
  if (mutex_ != NULL) osMutexRelease(*mutex_);
}

/* Transmit instruction packet to Dynamixel */
void DynamixelSerial::transmitInstructionPacket(uint8_t id, uint16_t len, uint8_t instruction, uint8_t* parameters)
{
  uint8_t transmit_data[INSTRUCTION_PACKET_SIZE];
  transmit_data[0] = HEADER0; transmit_data[1] = HEADER1;
  transmit_data[2] = HEADER2; transmit_data[3] = HEADER3;
  transmit_data[4] = id;
  transmit_data[7] = instruction;
  unsigned int next = 8;
  for (unsigned int i = 0; i + 3 < len; ++i)
  {
    if (next + 3 >= sizeof(transmit_data)) return;
    transmit_data[next++] = parameters[i];
    if (next >= 10 && transmit_data[next - 3] == HEADER0 && transmit_data[next - 2] == HEADER1 &&
        transmit_data[next - 1] == HEADER2)
      transmit_data[next++] = EXCEPTION_ADDITIONAL_BYTE;
  }
  // Protocol length includes stuffed bytes; CRC is calculated on the wire representation.
  const uint16_t wire_len = next - 7 + 2;
  transmit_data[5] = wire_len & 0xff;
  transmit_data[6] = wire_len >> 8;
  const uint16_t crc = calcCRC16(0, transmit_data, next);
  transmit_data[next++] = crc & 0xff;
  transmit_data[next++] = crc >> 8;
  if(direct_ttl_mode_) HAL_HalfDuplex_EnableTransmitter(huart_);
  HAL_UART_Transmit(huart_, transmit_data, next, 10);
  // Blocking transmit waits for TC. Switch now, before a zero-delay servo reply can arrive.
  if(direct_ttl_mode_) HAL_HalfDuplex_EnableReceiver(huart_);
}
/* Receive status packet to Dynamixel */
void DynamixelSerial::discardRx()
{
  rd_ptr_ = (RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % RX_BUFFER_SIZE;
}

int DynamixelSerial::receiveStatusFrame(bool fast)
{
  const uint32_t start = HAL_GetTick();
  const uint8_t header[] = {HEADER0, HEADER1, HEADER2, HEADER3};
  unsigned int size = 0;
  unsigned int total = 7;
  while (size < total)
  {
    if (HAL_GetTick() - start >= 10) return -1; // bounded even with a continuous malformed stream
    uint8_t byte;
    if (read(&byte, 1) != HAL_OK) return -1;
    if (size < 4 && byte != header[size]) return -1;
    status_frame_[size++] = byte;
    if (size == 7)
    {
      total = 7 + (uint16_t)(status_frame_[5] | (uint16_t(status_frame_[6]) << 8));
      if (total < 11 || total > sizeof(status_frame_)) return -1;
      if (fast && (requested_count_ == 0 || total != 8 + requested_count_ * 14)) return -1;
    }
  }
  const uint16_t crc = status_frame_[total-2] | (uint16_t(status_frame_[total-1]) << 8);
  if (crc != calcCRC16(0, status_frame_, total-2)) return -1;
  if (fast) return total; // Fast Sync status has NO byte stuffing (ROBOTIS Protocol 2.0).

  unsigned int out = 7;
  for (unsigned int in = 7; in < total-2; ++in)
  {
    status_frame_[out++] = status_frame_[in];
    if (out >= 10 && status_frame_[out-3] == HEADER0 &&
        status_frame_[out-2] == HEADER1 && status_frame_[out-1] == HEADER2)
    {
      if (++in >= total-2 || status_frame_[in] != EXCEPTION_ADDITIONAL_BYTE) return -1;
    }
  }
  return out; // unstuffed header/instruction/error/parameters, excluding CRC
}

int8_t DynamixelSerial::readStatusPacket(uint8_t status_packet_instruction)
{
  const int size = receiveStatusFrame(false);
  if (size < 9 || status_frame_[7] != STATUS_PACKET_INSTRUCTION ||
      (status_frame_[8] & 0x7f) != ERROR_NO_ERROR) return -1;
  const uint8_t servo_id = status_frame_[4];
  if (status_packet_instruction == INST_PING)
  {
    if (size != 12 || servo_id > 252 || servo_num_ >= MAX_SERVO_NUM) return -1;
    if (std::find(servo_.begin(), servo_.end(), ServoData(servo_id)) != servo_.end()) return -1;
  }
  else
  {
    if (response_index_ >= requested_count_ || servo_id != requested_ids_[response_index_] ||
        size != 9 + requested_size_) return -1;
    ++response_index_;
  }
  return applyStatusData(status_packet_instruction, servo_id, status_frame_ + 9);
}

int8_t DynamixelSerial::readFastSyncStatusPacket()
{
  const int size = receiveStatusFrame(true);
  if (size < 0 || status_frame_[4] != DX_BROADCAST_ID ||
      status_frame_[7] != STATUS_PACKET_INSTRUCTION) return -1;
  // Validate the entire chain before modifying any servo. The final CRC covers the
  // aggregate packet, including intermediate CRCs; no byte unstuffing is performed.
  for (unsigned int i = 0; i < requested_count_; ++i)
  {
    const unsigned int offset = 8 + i * 14;
    if ((status_frame_[offset] & 0x7f) != ERROR_NO_ERROR ||
        status_frame_[offset+1] != requested_ids_[i]) return -1;
  }
  for (unsigned int i = 0; i < requested_count_; ++i)
    if (applyStatusData(INST_GET_CUR_VEL_POS, requested_ids_[i], status_frame_ + 10 + i * 14) != 0)
      return -1;
  return 0;
}

int8_t DynamixelSerial::applyStatusData(uint8_t status_packet_instruction, uint8_t servo_id, const uint8_t* parameters)
{
  auto s = std::find(servo_.begin(), servo_.begin() + servo_num_, ServoData(servo_id));
  if (status_packet_instruction != INST_PING && s == servo_.begin() + servo_num_) return -1;
	switch (status_packet_instruction) {
	case INST_PING:
		servo_[servo_num_++].id_ = servo_id;
	    return 0;
  case INST_GET_CUR_VEL_POS:
  {
    // Preserve the position path's internal/external encoder and offset handling.
    if (applyStatusData(INST_GET_PRESENT_POS, servo_id, parameters + 6) != 0) return -1;
    s->present_current_ = int16_t(uint16_t(parameters[0]) | (uint16_t(parameters[1]) << 8));
    const uint32_t raw = uint32_t(parameters[2]) | (uint32_t(parameters[3]) << 8) |
                         (uint32_t(parameters[4]) << 16) | (uint32_t(parameters[5]) << 24);
    // Copy the signed two's-complement bits without signed-shift overflow.
    memcpy(&s->present_velocity_, &raw, sizeof(raw));
    s->feedback_fresh_ = true;
    return 0;
  }
	case INST_GET_PRESENT_POS:
	{
		const uint32_t raw_position = uint32_t(parameters[0]) | (uint32_t(parameters[1]) << 8) |
                                      (uint32_t(parameters[2]) << 16) | (uint32_t(parameters[3]) << 24);
        int32_t present_position;
        memcpy(&present_position, &raw_position, sizeof(raw_position));
		if (s != servo_.end()) {
                  s->hardware_error_status_ &= ((1 << ENCODER_CONNECT_ERROR) - 1); // &= 0b01111111
                  if(s->external_encoder_flag_) {
#ifndef SPINAL
                    encoder_handler_.update();
                    if(encoder_handler_.connected()) {
                      s->present_position_ = (int32_t)(encoder_handler_.getValue()); // use external encoder value instead of servo internal encoder value

                      if (s->first_get_pos_flag_) {
                        s->internal_offset_ = s->resolution_ratio_ * s->present_position_ - present_position;
                        s->first_get_pos_flag_ = false;
                      }

                      // TODO: check tooth jump
                    }
                    else {
                      s->hardware_error_status_ |= 1 << ENCODER_CONNECT_ERROR; // |= 0b10000000:  encoder is not connected
                      return -1;
                    }
#else
                    return -1; // no external encoder sample is available on Spinal
#endif
                  }
                  else {
                    if (s->first_get_pos_flag_) {
                      s->internal_offset_ = std::floor(present_position / 4096.0) * -4096; // to convert [0, 4096]
                      s->first_get_pos_flag_ = false;
                    }
                    s->setPresentPosition(present_position);
                  }
#if !STM32H7_V2
                  s->feedback_fresh_ = true;
#endif
                  // V2 only marks the combined current/velocity/position sample fresh.
                  // A position-only calibration read must not republish an old velocity.
		}
                return 0;
	}
	case INST_GET_PRESENT_CURRENT:
		if (s != servo_.end()) {
			s->present_current_ = ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return 0;
	case INST_GET_PRESENT_TEMPERATURE:
		if (s != servo_.end()) {
			s->present_temp_ = parameters[0];
		}
		return 0;
	case INST_GET_PRESENT_MOVING:
		if (s != servo_.end()) {
			s->moving_ = parameters[0];
		}
		return 0;
	case INST_GET_HOMING_OFFSET:
		if (s != servo_.end()) {
			s->homing_offset_ = ((parameters[3] << 24) & 0xFF000000) | ((parameters[2] << 16) & 0xFF0000) | ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return 0;
	case INST_GET_HARDWARE_ERROR_STATUS:
		if (s != servo_.end()) {
                  s->hardware_error_status_ &=  ((1 << RESOLUTION_RATIO_ERROR) + (1 << ENCODER_CONNECT_ERROR));  //  &= 0b11000000;
                  s->hardware_error_status_ |= parameters[0];
		}
		return 0;
	case INST_GET_CURRENT_LIMIT:
		if (s != servo_.end()) {
			s->current_limit_ = ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return 0;
	case INST_GET_POSITION_GAINS:
		if (s != servo_.end()) {
			s->d_gain_ = ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
			s->i_gain_ = ((parameters[3] << 8) & 0xFF00) | (parameters[2] & 0xFF);
			s->p_gain_ = ((parameters[5] << 8) & 0xFF00) | (parameters[4] & 0xFF);
		}
		return 0;
	case INST_GET_PROFILE_VELOCITY:
		if (s != servo_.end()) {
			s->profile_velocity_ = ((parameters[3] << 24) & 0xFF000000) | ((parameters[2] << 16) & 0xFF0000) | ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return 0;
	default:
		return -1;
	}
}

void DynamixelSerial::cmdPing(uint8_t id)
{
  transmitInstructionPacket(id, PING_LEN, COMMAND_PING, nullptr);
}

void DynamixelSerial::cmdReboot(uint8_t id)
{
	transmitInstructionPacket(id, REBOOT_LEN, COMMAND_REBOOT, nullptr);
}

void DynamixelSerial::cmdRead(uint8_t id, uint16_t address, uint16_t byte_size)
{
  discardRx();
  requested_ids_[0] = id;
  requested_count_ = 1;
  response_index_ = 0;
  requested_size_ = byte_size;
  fast_read_pending_ = false;
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = byte_size & 0xFF;
	parameters[3] = (byte_size >> 8) & 0xFF;
	transmitInstructionPacket(id, READ_INST_LEN, COMMAND_READ, parameters);
}

void DynamixelSerial::cmdWrite(uint8_t id, uint16_t address, uint8_t* param, int param_len)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	for (int i = 0; i < param_len; i++) {
		parameters[i + 2] = param[i];
	}
	transmitInstructionPacket(id, param_len + 5, COMMAND_WRITE, parameters);
}

void DynamixelSerial::cmdSyncRead(uint16_t address, uint16_t byte_size, bool send_all)
{
  discardRx();
  requested_count_ = 0;
  response_index_ = 0;
  requested_size_ = byte_size;
  fast_read_pending_ = false;
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = byte_size & 0xFF;
	parameters[3] = (byte_size >> 8) & 0xFF;
	int param_idx = 4;
	for (unsigned int i = 0; i < servo_num_; i++) {
          if(!send_all && !servo_[i].send_data_flag_ && !servo_[i].first_get_pos_flag_) continue;
          parameters[param_idx++] = servo_[i].id_;
          requested_ids_[requested_count_++] = servo_[i].id_;
	}
  if (!requested_count_) return;
#if DYNAMIXEL_USE_FAST_SYNC_READ
  fast_read_pending_ = address == CTRL_PRESENT_CURRENT && byte_size == PRESENT_CUR_VEL_POS_BYTE_LEN;
#endif
  transmitInstructionPacket(DX_BROADCAST_ID, param_idx + 3,
                            fast_read_pending_ ? COMMAND_FAST_SYNC_READ : COMMAND_SYNC_READ, parameters);
}

void DynamixelSerial::cmdSyncWrite(uint16_t address, uint8_t* param, int param_len)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = param_len & 0xFF;
	parameters[3] = (param_len >> 8) & 0xFF;
	int all_param_idx = 4;
	int param_idx = 0;
	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[all_param_idx++] = servo_[i].id_;
		for (int j = 0; j < param_len; j++) {
			parameters[all_param_idx++] = param[param_idx++];
		}
	}
	transmitInstructionPacket(DX_BROADCAST_ID, all_param_idx + 3, COMMAND_SYNC_WRITE, parameters);
}

void DynamixelSerial::cmdReadCurrentLimit(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_CURRENT_LIMIT, CURRENT_LIMIT_BYTE_LEN);
}

void DynamixelSerial::cmdReadHardwareErrorStatus(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_HARDWARE_ERROR_STATUS, HARDWARE_ERROR_STATUS_BYTE_LEN);
}

void DynamixelSerial::cmdReadHomingOffset(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_HOMING_OFFSET, HOMING_OFFSET_BYTE_LEN);
}

void DynamixelSerial::cmdReadMoving(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_MOVING, MOVING_BYTE_LEN);
}

void DynamixelSerial::cmdReadPositionGains(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_POSITION_D_GAIN, POSITION_GAINS_BYTE_LEN);
}

void DynamixelSerial::cmdReadPresentCurrent(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PRESENT_CURRENT, PRESENT_CURRENT_BYTE_LEN);
}

void DynamixelSerial::cmdReadPresentCurVelPos(uint8_t servo_index)
{
  cmdRead(servo_[servo_index].id_, CTRL_PRESENT_CURRENT, PRESENT_CUR_VEL_POS_BYTE_LEN);
}

void DynamixelSerial::cmdSyncReadPresentCurVelPos()
{
  cmdSyncRead(CTRL_PRESENT_CURRENT, PRESENT_CUR_VEL_POS_BYTE_LEN, false);
}

void DynamixelSerial::cmdReadPresentPosition(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PRESENT_POSITION, PRESENT_POSITION_BYTE_LEN);
}

void DynamixelSerial::cmdReadPresentTemperature(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PRESENT_TEMPERATURE, PRESENT_TEMPERATURE_BYTE_LEN);
}

void DynamixelSerial::cmdReadProfileVelocity(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PROFILE_VELOCITY, PROFILE_VELOCITY_BYTE_LEN);
}

void DynamixelSerial::cmdWriteCurrentLimit(uint8_t servo_index)
{
	uint16_t current_limit = servo_[servo_index].current_limit_;
	uint8_t parameters[CURRENT_LIMIT_BYTE_LEN];
	parameters[0] = (uint8_t)(current_limit & 0xFF);
	parameters[1] = (uint8_t)((current_limit >> 8) & 0xFF);

	cmdWrite(servo_[servo_index].id_, CTRL_CURRENT_LIMIT, parameters, CURRENT_LIMIT_BYTE_LEN);
}

void DynamixelSerial::cmdWriteHomingOffset(uint8_t servo_index)
{
	int32_t homing_offset = servo_[servo_index].homing_offset_;
	uint8_t parameters[4];
	parameters[0] = (uint8_t)(homing_offset & 0xFF);
	parameters[1] = (uint8_t)((homing_offset >> 8) & 0xFF);
	parameters[2] = (uint8_t)((homing_offset >> 16) & 0xFF);
	parameters[3] = (uint8_t)((homing_offset >> 24) & 0xFF);
	cmdWrite(servo_[servo_index].id_, CTRL_HOMING_OFFSET, parameters, HOMING_OFFSET_BYTE_LEN);
}

void DynamixelSerial::cmdWritePositionGains(uint8_t servo_index)
{
	uint8_t parameters[POSITION_GAINS_BYTE_LEN];
	parameters[0] = servo_[servo_index].d_gain_ & 0xFF;
	parameters[1] = (servo_[servo_index].d_gain_ >> 8) & 0xFF;
	parameters[2] = servo_[servo_index].i_gain_ & 0xFF;
	parameters[3] = (servo_[servo_index].i_gain_ >> 8) & 0xFF;
	parameters[4] = servo_[servo_index].p_gain_ & 0xFF;
	parameters[5] = (servo_[servo_index].p_gain_ >> 8) & 0xFF;

	cmdWrite(servo_[servo_index].id_, CTRL_POSITION_D_GAIN, parameters, POSITION_GAINS_BYTE_LEN);
}

void DynamixelSerial::cmdWriteProfileVelocity(uint8_t servo_index)
{
	uint16_t vel = servo_[servo_index].profile_velocity_;
	uint8_t parameters[PROFILE_VELOCITY_BYTE_LEN];
	parameters[0] = (uint8_t)(vel & 0xFF);
	parameters[1] = (uint8_t)((vel >> 8) & 0xFF);
	parameters[2] = (uint8_t)((vel >> 16) & 0xFF);
	parameters[3] = (uint8_t)((vel >> 24) & 0xFF);
	cmdWrite(servo_[servo_index].id_, CTRL_PROFILE_VELOCITY, parameters, PROFILE_VELOCITY_BYTE_LEN);
}

void DynamixelSerial::cmdWriteStatusReturnLevel(uint8_t id, uint8_t set)
{
	uint8_t param = set;
	cmdWrite(id, CTRL_STATUS_RETURN_LEVEL, &param, STATUS_RETURN_LEVEL_BYTE_LEN);
}

void DynamixelSerial::cmdWriteTorqueEnable(uint8_t servo_index)
{

	uint8_t	parameter = (uint8_t)((servo_[servo_index].torque_enable_) ? 1 : 0);

	cmdWrite(servo_[servo_index].id_, CTRL_TORQUE_ENABLE, &parameter, TORQUE_ENABLE_BYTE_LEN);
}

void DynamixelSerial::cmdSyncReadCurrentLimit(bool send_all)
{
	cmdSyncRead(CTRL_CURRENT_LIMIT, CURRENT_LIMIT_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadHardwareErrorStatus(bool send_all)
{
	cmdSyncRead(CTRL_HARDWARE_ERROR_STATUS, HARDWARE_ERROR_STATUS_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadHomingOffset(bool send_all)
{
	cmdSyncRead(CTRL_HOMING_OFFSET, HOMING_OFFSET_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadMoving(bool send_all)
{
	cmdSyncRead(CTRL_MOVING, MOVING_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadPositionGains(bool send_all)
{
	cmdSyncRead(CTRL_POSITION_D_GAIN, POSITION_GAINS_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadPresentCurrent(bool send_all)
{
	cmdSyncRead(CTRL_PRESENT_CURRENT, PRESENT_CURRENT_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadPresentPosition(bool send_all)
{
	cmdSyncRead(CTRL_PRESENT_POSITION, PRESENT_POSITION_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadPresentTemperature(bool send_all)
{
	cmdSyncRead(CTRL_PRESENT_TEMPERATURE, PRESENT_TEMPERATURE_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadProfileVelocity(bool send_all)
{
	cmdSyncRead(CTRL_PROFILE_VELOCITY, PROFILE_VELOCITY_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncWriteGoalPosition()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		int32_t goal_position = servo_[i].getGoalPosition();
		parameters[i * 4 + 0] = (uint8_t)((int32_t)(goal_position) & 0xFF);
		parameters[i * 4 + 1] = (uint8_t)(((int32_t)(goal_position) >> 8) & 0xFF);
		parameters[i * 4 + 2] = (uint8_t)(((int32_t)(goal_position) >> 16) & 0xFF);
		parameters[i * 4 + 3] = (uint8_t)(((int32_t)(goal_position) >> 24) & 0xFF);
	}

	cmdSyncWrite(CTRL_GOAL_POSITION, parameters, GOAL_POSITION_BYTE_LEN);
}

void DynamixelSerial::cmdSyncWriteLed()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i] = (uint8_t)((servo_[i].led_) ? 1 : 0);
	}

	cmdSyncWrite(CTRL_LED, parameters, LED_BYTE_LEN);
}

void DynamixelSerial::cmdSyncWritePositionGains()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
        /*p_gain*/
        for (unsigned int i = 0; i < servo_num_; i++) {
          parameters[i * 2 + 0] = servo_[i].p_gain_ & 0xFF;
          parameters[i * 2 + 1] = (servo_[i].p_gain_ >> 8) & 0xFF;
	}
	cmdSyncWrite(CTRL_POSITION_P_GAIN, parameters, 2);

        /*i_gain*/
        for (unsigned int i = 0; i < servo_num_; i++) {
          parameters[i * 2 + 0] = servo_[i].i_gain_ & 0xFF;
          parameters[i * 2 + 1] = (servo_[i].i_gain_ >> 8) & 0xFF;
	}
	cmdSyncWrite(CTRL_POSITION_I_GAIN, parameters, 2);

        /*d_gain*/
        for (unsigned int i = 0; i < servo_num_; i++) {
          parameters[i * 2 + 0] = servo_[i].d_gain_ & 0xFF;
          parameters[i * 2 + 1] = (servo_[i].d_gain_ >> 8) & 0xFF;
	}
	cmdSyncWrite(CTRL_POSITION_D_GAIN, parameters, 2);
}

void DynamixelSerial::cmdSyncWriteProfileVelocity()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i * 4 + 0] = (uint8_t)((int32_t)(servo_[i].profile_velocity_) & 0xFF);
		parameters[i * 4 + 1] = (uint8_t)(((int32_t)(servo_[i].profile_velocity_) >> 8) & 0xFF);
		parameters[i * 4 + 2] = (uint8_t)(((int32_t)(servo_[i].profile_velocity_) >> 16) & 0xFF);
		parameters[i * 4 + 3] = (uint8_t)(((int32_t)(servo_[i].profile_velocity_) >> 24) & 0xFF);
	}

	cmdSyncWrite(CTRL_PROFILE_VELOCITY, parameters, PROFILE_VELOCITY_BYTE_LEN);
}

void DynamixelSerial::cmdSyncWriteTorqueEnable()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i] = (uint8_t)((servo_[i].torque_enable_) ? 1 : 0);
	}

	cmdSyncWrite(CTRL_TORQUE_ENABLE, parameters, TORQUE_ENABLE_BYTE_LEN);
}


void DynamixelSerial::setStatusReturnLevel()
{
	cmdWriteStatusReturnLevel(DX_BROADCAST_ID, READ);
}

void DynamixelSerial::getHomingOffset()
{
	if (ttl_rs485_mixed_ != 0) {
		for (unsigned int i = 0; i < servo_num_; ++i) {
			cmdReadHomingOffset(i);
			readStatusPacket(INST_GET_HOMING_OFFSET);
		}
	} else {
		cmdSyncReadHomingOffset();
		for (unsigned int i = 0; i < servo_num_; i++) {
			readStatusPacket(INST_GET_HOMING_OFFSET);
		}
	}
}

void DynamixelSerial::getCurrentLimit()
{
	if (ttl_rs485_mixed_ != 0) {
		for (unsigned int i = 0; i < servo_num_; ++i) {
			cmdReadCurrentLimit(i);
			readStatusPacket(INST_GET_CURRENT_LIMIT);
		}
	} else {
		cmdSyncReadCurrentLimit();
		for (unsigned int i = 0; i < servo_num_; i++) {
			readStatusPacket(INST_GET_CURRENT_LIMIT);
		}
	}
}

void DynamixelSerial::getPositionGains()
{
	if (ttl_rs485_mixed_ != 0) {
		for (unsigned int i = 0; i < servo_num_; ++i) {
			cmdReadPositionGains(i);
			readStatusPacket(INST_GET_POSITION_GAINS);
		}
	} else {
		cmdSyncReadPositionGains();
		for (unsigned int i = 0; i < servo_num_; i++) {
			readStatusPacket(INST_GET_POSITION_GAINS);
		}
	}
}

void DynamixelSerial::getProfileVelocity()
{
	if (ttl_rs485_mixed_ != 0) {
		for (unsigned int i = 0; i < servo_num_; ++i) {
			cmdReadProfileVelocity(i);
			readStatusPacket(INST_GET_PROFILE_VELOCITY);
		}
	} else {
		cmdSyncReadProfileVelocity();
		for (unsigned int i = 0; i < servo_num_; i++) {
			readStatusPacket(INST_GET_PROFILE_VELOCITY);
		}
	}
}

static const uint16_t crc_table[256] = {
          0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
          0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
          0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
          0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
          0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
          0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
          0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
          0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
          0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
          0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
          0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
          0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
          0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
          0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
          0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
          0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
          0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
          0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
          0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
          0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
          0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
          0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
          0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
          0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
          0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
          0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
          0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
          0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
          0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
          0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
          0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
          0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
      	};

uint16_t DynamixelSerial::calcCRC16(uint16_t crc_accum, uint8_t *data_blk_ptr, int data_blk_size)
{
	for(int j = 0; j < data_blk_size; j++) {
		int i = ((int)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}
	return crc_accum;
}

HAL_StatusTypeDef DynamixelSerial::read(uint8_t* data,  uint32_t timeout)
{
  /* handle RX Overrun Error */
  if ( __HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) )
    {
      HAL_UART_AbortReceive(huart_); // abort clears flags and flushes the UART receive register
      rd_ptr_ = 0;
      memset(rx_buf_, 0, RX_BUFFER_SIZE);
      HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE);
      __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
      __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
      return HAL_ERROR;
    }

  uint32_t tick_start = HAL_GetTick();

  while (true)
    {
      // uint32_t dma_write_ptr =  (RX_BUFFER_SIZE - huart_->hdmarx->Instance->NDTR) % (RX_BUFFER_SIZE);
      uint32_t dma_write_ptr =  (RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (RX_BUFFER_SIZE);
      if(rd_ptr_ != dma_write_ptr)
        {
          *data = (int)rx_buf_[rd_ptr_++];
          rd_ptr_ %= RX_BUFFER_SIZE;
          return HAL_OK;
        }

      if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
        {
          return HAL_TIMEOUT;
        }
    }
}
