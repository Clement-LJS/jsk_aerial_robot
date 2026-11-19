#include "dynamixel_serial.h"
#include "dynamixel_packet.h"
#include <limits.h>
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

  pinReconfig();

        /* rx */
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  #if DYNAMIXEL_BOARDLESS_CONTROL
    HAL_HalfDuplex_EnableReceiver(huart_);
  #endif
  HAL_UART_Receive_DMA(huart, rx_buf_, RX_BUFFER_SIZE);
  rd_ptr_ = 0;
  memset(rx_buf_, 0, sizeof(rx_buf_));

	std::fill(servo_.begin(), servo_.end(), ServoData(255));

	//initialize servo motors
	HAL_Delay(500);
	ping();
	HAL_Delay(500);
	for (unsigned int i = 0; i < servo_num_; i++) {
		reboot(i);
	}
	HAL_Delay(2000);

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
  // Erased/invalid storage defaults to individual reads, which work across
  // TTL and RS485 bridges. Preserve an explicitly saved 0 or 1.
  if (ttl_rs485_mixed_ > 1) ttl_rs485_mixed_ = 1;

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
          if(servo_[i].servo_resolution_ == 65535 || servo_[i].joint_resolution_ == 65535 ||
             servo_[i].servo_resolution_ == 0 || servo_[i].joint_resolution_ == 0){
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
  while(HAL_UART_DeInit(huart_) != HAL_OK);
  /*Change baud rate*/
  huart_->Init.BaudRate = 1000000;
  huart_->Init.WordLength = UART_WORDLENGTH_8B;
  huart_->Init.Parity = UART_PARITY_NONE;
  huart_->Init.Mode = UART_MODE_TX_RX;
  /*Initialize as halfduplex mode*/
#if DYNAMIXEL_BOARDLESS_CONTROL
  while(HAL_HalfDuplex_Init(huart_) != HAL_OK);  
#else 
  while(HAL_UART_Init(huart_) != HAL_OK);
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
  instruction_buffer_.push(std::make_pair(INST_SET_TORQUE, servo_index));
}

void DynamixelSerial::setTorqueFromPresetnPos(uint8_t servo_index)
{
  ServoData& s = servo_[servo_index];
  // Present positions include the runtime wrap offset; motor goals do not.
  if(s.torque_enable_) s.setGoalPosition(s.present_position_);
  instruction_buffer_.push(std::make_pair(INST_SET_TORQUE, servo_index));
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
  else {
    calibrateHomingOffset(servo_index, servo_[servo_index].calib_value_);
  }
}

bool DynamixelSerial::readRegister(uint8_t index, uint16_t address, uint16_t size)
{
  if (index >= servo_num_ || size > sizeof(register_data_)) return false;
  cmdRead(servo_[index].id_, address, size);
  return readStatusPacket(INST_GET_REGISTER) == 0;
}

bool DynamixelSerial::calibrateHomingOffset(uint8_t index, int32_t target)
{
  if (index >= servo_num_ || target < 0 || target > 4095) return false;
  ServoData& s = servo_[index];
  if (s.external_encoder_flag_ || s.torque_enable_) return false;
  if (mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
  // Use fresh hardware values, including torque state. Never zero the EEPROM
  // offset before establishing a valid position and calculating its replacement.
  bool ok = readRegister(index, CTRL_TORQUE_ENABLE, 1) && register_data_[0] == 0;
  int32_t original_offset = 0;
  int32_t raw = 0;
  if (ok) {
    ok = readRegister(index, CTRL_HOMING_OFFSET, 4);
    original_offset = dynamixel_packet::signed32(register_data_);
  }
  if (ok) {
    ok = readRegister(index, CTRL_PRESENT_POSITION, 4);
    raw = dynamixel_packet::signed32(register_data_);
  }
  const int64_t candidate = static_cast<int64_t>(original_offset) + target - dynamixel_packet::singleTurn(raw);
  ok = ok && candidate >= INT32_MIN && candidate <= INT32_MAX;
  if (ok) {
    s.homing_offset_ = static_cast<int32_t>(candidate);
    cmdWriteHomingOffset(index);
    // EEPROM writes can take longer than a normal register read.
    HAL_Delay(20);
    ok = readRegister(index, CTRL_HOMING_OFFSET, 4);
    if (ok) {
      s.homing_offset_ = dynamixel_packet::signed32(register_data_);
      ok = s.homing_offset_ == candidate;
    }
    if (ok) {
      ok = readRegister(index, CTRL_PRESENT_POSITION, 4);
      raw = dynamixel_packet::signed32(register_data_);
      const int32_t delta = dynamixel_packet::singleTurn(raw) - target;
      ok = ok && delta >= -2 && delta <= 2;
    }
    // The next valid sample re-establishes the same single-turn origin as boot.
    s.first_get_pos_flag_ = true;
    s.position_valid_ = false;
    s.position_updated_ = false;
  }
  if (mutex_ != NULL) osMutexRelease(*mutex_);
  return ok;
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

void DynamixelSerial::update()
{
  uint32_t current_time = HAL_GetTick();

  /* send position command to servo */
  if(current_time >= set_pos_tick_ + SET_POS_DU && SET_POS_DU > 0) {
    if (set_pos_tick_ == 0) set_pos_tick_ = current_time + SET_POS_OFFSET; // init
    else set_pos_tick_ = current_time;

    // A sync write already includes every motor, including in mixed mode.
    instruction_buffer_.push(std::make_pair(INST_SET_GOAL_POS, 0));
  }

  /* read servo position(angle) */
  if(current_time >= get_pos_tick_ + GET_POS_DU && GET_POS_DU > 0) {
    if (get_pos_tick_ == 0) get_pos_tick_ = current_time + GET_POS_OFFSET; // init
    else get_pos_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_POS, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_POS, 0));
    }
  }

  /* read servo load */

  if(current_time >= get_load_tick_ + GET_LOAD_DU && GET_LOAD_DU > 0) {
    if (get_load_tick_ == 0) get_load_tick_ = current_time + GET_LOAD_OFFSET; // init
    else get_load_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_CURRENT, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_CURRENT, 0));
    }
  }

  /* read servo temperature */

  if(current_time >= get_temp_tick_ + GET_TEMP_DU && GET_TEMP_DU > 0) {
    if (get_temp_tick_ == 0) get_temp_tick_ = current_time + GET_TEMP_OFFSET;  // init
    else get_temp_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_TEMPERATURE, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_TEMPERATURE, 0));
    }
  }

  /* read servo movement */
  if(current_time >= get_move_tick_ + GET_MOVE_DU && GET_MOVE_DU > 0) {
    if (get_move_tick_ == 0) get_move_tick_ = current_time + GET_MOVE_OFFSET; // init
    else get_move_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_MOVING, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_MOVING, 0));
    }
  }

  /* read hardware error status */
  if(current_time >= get_error_tick_ + GET_HARDWARE_ERROR_STATUS_DU && GET_HARDWARE_ERROR_STATUS_DU > 0) {
    if (get_error_tick_ == 0) get_error_tick_ = current_time + GET_HARDWARE_ERROR_STATUS_OFFSET; // init
    else get_error_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_HARDWARE_ERROR_STATUS, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_HARDWARE_ERROR_STATUS, 0));
    }
  }


  /* process the instruction from the instruction buffer */
  std::pair<uint8_t, uint8_t> instruction;
  while(instruction_buffer_.pop(instruction))
    {
      uint8_t servo_index = instruction.second;

      if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

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

      bool read_status_packet_flag = false;
      /* get command */
      if (ttl_rs485_mixed_ != 0) {
        switch (instruction.first) {
        case INST_GET_PRESENT_POS: /* read servo position(angle) */
          if(!servo_[servo_index].send_data_flag_ && !servo_[servo_index].first_get_pos_flag_) break;
          cmdReadPresentPosition(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_PRESENT_CURRENT: /* read servo load */
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadPresentCurrent(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_PRESENT_TEMPERATURE: /* read servo temp */
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadPresentTemperature(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_PRESENT_MOVING: /* read servo movement */
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadMoving(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_HARDWARE_ERROR_STATUS:
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadHardwareErrorStatus(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_HOMING_OFFSET:
          for (unsigned int i = 0; i < servo_num_; ++i) {
            cmdReadHomingOffset(i);
            readStatusPacket(instruction.first);
          }
          break;
        case INST_GET_POSITION_GAINS:
          for (unsigned int i = 0; i < servo_num_; ++i) {
            cmdReadPositionGains(i);
            readStatusPacket(instruction.first);
          }
          break;
        case INST_GET_PROFILE_VELOCITY:
          for (unsigned int i = 0; i < servo_num_; ++i) {
            cmdReadProfileVelocity(i);
            readStatusPacket(instruction.first);
          }
          break;
        case INST_GET_CURRENT_LIMIT:
          for (unsigned int i = 0; i < servo_num_; ++i) {
            cmdReadCurrentLimit(i);
            readStatusPacket(instruction.first);
          }
          break;
        default:
          break;
        }
      } else {
        switch (instruction.first) {
        case INST_GET_PRESENT_POS: /* read servo position(angle) */
          cmdSyncReadPresentPosition(false);
          read_status_packet_flag = true;
          break;
        case INST_GET_PRESENT_CURRENT: /* read servo load */
          cmdSyncReadPresentCurrent(false);
          read_status_packet_flag = true;
          break;
        case INST_GET_PRESENT_TEMPERATURE: /* read servo temp */
          cmdSyncReadPresentTemperature(false);
          read_status_packet_flag = true;
          break;
        case INST_GET_PRESENT_MOVING: /* read servo movement */
          cmdSyncReadMoving(false);
          read_status_packet_flag = true;
          break;
        case INST_GET_HARDWARE_ERROR_STATUS:
          cmdSyncReadHardwareErrorStatus(false);
          read_status_packet_flag = true;
          break;
        case INST_GET_HOMING_OFFSET:
          cmdSyncReadHomingOffset(false);
          read_status_packet_flag = true;
          break;
        case INST_GET_POSITION_GAINS:
          cmdSyncReadPositionGains(false);
          read_status_packet_flag = true;
          break;
        case INST_GET_PROFILE_VELOCITY:
          cmdSyncReadProfileVelocity(false);
          read_status_packet_flag = true;
          break;
        case INST_GET_CURRENT_LIMIT:
          cmdSyncReadCurrentLimit(false);
          read_status_packet_flag = true;
          break;
        default:
          break;
        }

        if (read_status_packet_flag) {
          // Finish this batch before sending any other register request.
          for (unsigned int i = 0; expected_replies_ && i < servo_num_; ++i) {
            if (readStatusPacket(instruction.first) != 0) break;
          }
        }
      }

      if (mutex_ != NULL) osMutexRelease(*mutex_);
    }
}

/* Transmit instruction packet to Dynamixel */
void DynamixelSerial::transmitInstructionPacket(uint8_t id, uint16_t len, uint8_t instruction, uint8_t* parameters)
{
  uint8_t transmit_data[INSTRUCTION_PACKET_SIZE];
  if (len < 3) return;
  const size_t transmit_data_index = dynamixel_packet::encode(
      id, instruction, parameters, len - 3, transmit_data, sizeof(transmit_data));
  if (!transmit_data_index) return;

  /* send data */

#if DYNAMIXEL_BOARDLESS_CONTROL
  HAL_HalfDuplex_EnableTransmitter(huart_);
  uint8_t ret;
  ret = HAL_UART_Transmit(huart_, transmit_data, transmit_data_index, 10); //timeout: 10 ms. Although we found 2 ms is enough OK for our case by oscilloscope. Large value is better for UART async task in RTOS.
  if(ret == HAL_OK)
  {
    while (__HAL_UART_GET_FLAG(huart_, UART_FLAG_TC) == RESET) {}
    // After transmitting, enable the receiver
    HAL_HalfDuplex_EnableReceiver(huart_);
  }
#else
// WE; 
  HAL_UART_Transmit(huart_, transmit_data, transmit_data_index, 10); //timeout: 10 ms. Although we found 2 ms is enough OK for our case by oscilloscope. Large value is better for UART async task in RTOS.
// RE;
#endif
}
/* Receive status packet to Dynamixel */
int8_t DynamixelSerial::readStatusPacket(uint8_t status_packet_instruction)
{
  uint8_t packet[STATUS_PACKET_SIZE];
  uint8_t body[STATUS_PACKET_SIZE];
  size_t used = 0;
  size_t total = 0;
  const uint32_t start = HAL_GetTick();
  const uint8_t header[] = {0xff, 0xff, 0xfd, 0};
  while (HAL_GetTick() - start < 5) {
    uint8_t byte;
    if (read(&byte, 1) != HAL_OK) break;
    if (used < 4) {
      if (byte == header[used]) packet[used++] = byte;
      else if (used == 2 && byte == 0xff) { /* retain FF FF */ }
      else { used = byte == 0xff ? 1 : 0; packet[0] = byte; }
      continue;
    }
    packet[used++] = byte;
    if (used == 7) {
      total = 7u + packet[5] + (static_cast<size_t>(packet[6]) << 8);
      if (total < 11 || total > sizeof(packet)) { used = 0; continue; }
    }
    if (used < 7 || used != total) continue;
    const size_t count = dynamixel_packet::decodeStatus(packet, total, body, sizeof(body));
    const uint8_t servo_id = packet[4];
    auto s = std::find(servo_.begin(), servo_.begin() + servo_num_, ServoData(servo_id));
    if (status_packet_instruction == INST_PING) {
      if (count == 5 && s == servo_.begin() + servo_num_ && servo_num_ < MAX_SERVO_NUM) {
        servo_[servo_num_++].id_ = servo_id;
        return 0;
      }
    } else if (count == expected_size_ + 2u && s != servo_.begin() + servo_num_ &&
               (expected_replies_ & (1u << (s - servo_.begin())))) {
      expected_replies_ &= ~(1u << (s - servo_.begin()));
      const uint8_t* parameters = body + 2;

	/* read success */
	switch (status_packet_instruction) {
	case INST_GET_REGISTER:
        memcpy(register_data_, parameters, expected_size_);
        return 0;
	case INST_GET_PRESENT_POS:
	{
		int32_t present_position = dynamixel_packet::signed32(parameters);
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
                    }
#else
                    s->position_valid_ = false;
                    break;
#endif
                  }
                  else {
                    if (s->first_get_pos_flag_) {
                      const int64_t offset = static_cast<int64_t>(dynamixel_packet::singleTurn(present_position)) - present_position;
                      if (offset < INT32_MIN || offset > INT32_MAX) break;
                      s->internal_offset_ = static_cast<int32_t>(offset);
                      s->first_get_pos_flag_ = false;
                    }
                    const int64_t position = static_cast<int64_t>(present_position) + s->internal_offset_;
                    if (position < INT16_MIN || position > INT16_MAX) {
                      s->position_valid_ = false;
                      break; // ServoState.angle cannot represent this sample.
                    }
                    s->setPresentPosition(present_position);
                  }
                  s->raw_position_ = present_position;
                  s->position_tick_ = HAL_GetTick();
                  s->position_valid_ = !(s->hardware_error_status_ & (1u << ENCODER_CONNECT_ERROR));
                  s->position_updated_ = s->position_valid_;
                  if (s->position_valid_) setROSCommFlag(true);
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
			s->homing_offset_ = dynamixel_packet::signed32(parameters);
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
			s->profile_velocity_ = dynamixel_packet::signed32(parameters);
		}
		return 0;
	default:
		return -1;
	}
    }
    ++read_failures_; // malformed, duplicate, wrong-ID or wrong-size response
    used = 0;
  }
  ++read_failures_;
  recover_rx_ = true;
  return -1;
}

void DynamixelSerial::prepareRead()
{
  uint8_t discarded;
  // After an incomplete transaction, allow a bounded quiet interval before a
  // different request. Status packets contain no register address/transaction ID.
  const uint32_t start = HAL_GetTick();
  do {
    if (read(&discarded, recover_rx_ ? 2 : 0) != HAL_OK) break;
  } while (HAL_GetTick() - start < 5);
  recover_rx_ = false;
  expected_replies_ = 0;
}

void DynamixelSerial::cmdPing(uint8_t id)
{
  prepareRead();
  transmitInstructionPacket(id, PING_LEN, COMMAND_PING, nullptr);
}

void DynamixelSerial::cmdReboot(uint8_t id)
{
	transmitInstructionPacket(id, REBOOT_LEN, COMMAND_REBOOT, nullptr);
}

void DynamixelSerial::cmdRead(uint8_t id, uint16_t address, uint16_t byte_size)
{
  prepareRead();
  expected_size_ = byte_size;
  for (unsigned i = 0; i < servo_num_; ++i)
    if (servo_[i].id_ == id) expected_replies_ = 1u << i;
	uint8_t parameters[4];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = byte_size & 0xFF;
	parameters[3] = (byte_size >> 8) & 0xFF;
	transmitInstructionPacket(id, READ_INST_LEN, COMMAND_READ, parameters);
}

void DynamixelSerial::cmdWrite(uint8_t id, uint16_t address, uint8_t* param, int param_len)
{
  if (param_len < 0 || param_len > 6) return;
	uint8_t parameters[8];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	for (int i = 0; i < param_len; i++) {
		parameters[i + 2] = param[i];
	}
	transmitInstructionPacket(id, param_len + 5, COMMAND_WRITE, parameters);
}

void DynamixelSerial::cmdSyncRead(uint16_t address, uint16_t byte_size, bool send_all)
{
  prepareRead();
  expected_size_ = byte_size;
	uint8_t parameters[4 + MAX_SERVO_NUM];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = byte_size & 0xFF;
	parameters[3] = (byte_size >> 8) & 0xFF;
	int param_idx = 4;
	for (unsigned int i = 0; i < servo_num_; i++) {
          if(!send_all && !servo_[i].send_data_flag_ && !servo_[i].first_get_pos_flag_) continue;
          parameters[param_idx++] = servo_[i].id_;
          expected_replies_ |= 1u << i;
	}
	transmitInstructionPacket(DX_BROADCAST_ID, param_idx + 3, COMMAND_SYNC_READ, parameters);
}

void DynamixelSerial::cmdSyncWrite(uint16_t address, uint8_t* param, int param_len)
{
  if (param_len < 0 || 4u + servo_num_ * (param_len + 1u) > INSTRUCTION_PACKET_SIZE) return;
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
	uint8_t parameters[MAX_SERVO_NUM * 4];

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
	uint8_t parameters[MAX_SERVO_NUM];
	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i] = (uint8_t)((servo_[i].led_) ? 1 : 0);
	}

	cmdSyncWrite(CTRL_LED, parameters, LED_BYTE_LEN);
}

void DynamixelSerial::cmdSyncWritePositionGains()
{
	uint8_t parameters[MAX_SERVO_NUM * 2];
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
	uint8_t parameters[MAX_SERVO_NUM * 4];

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
	uint8_t parameters[MAX_SERVO_NUM];

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

HAL_StatusTypeDef DynamixelSerial::read(uint8_t* data,  uint32_t timeout)
{
  /* handle RX Overrun Error */
  if ( __HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) )
    {
      __HAL_UART_CLEAR_FLAG(huart_, UART_FLAG_RXNE);
      __HAL_UART_CLEAR_PEFLAG(huart_);
      __HAL_UART_CLEAR_OREFLAG(huart_);
      __HAL_UART_CLEAR_FEFLAG(huart_);

      HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE);
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
