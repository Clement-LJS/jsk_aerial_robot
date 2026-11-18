//
// Created by li-jinjie on 24-1-21.
// ref: https://github.com/mokhwasomssi/stm32_hal_dshot/tree/main
//

#include "dshot.h"

// One DMA buffer per channel, defined once (not once per including translation unit).
namespace
{
#ifdef STM32H7
  uint32_t motor1_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection1")));
  uint32_t motor2_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection2")));
  uint32_t motor3_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection3")));
  uint32_t motor4_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection4")));
#else
  uint32_t motor1_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
  uint32_t motor2_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
  uint32_t motor3_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
  uint32_t motor4_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
#endif
}

void DShot::init(dshot_type_e dshot_type, TIM_HandleTypeDef* htim_motor_1, uint32_t channel_motor_1,
                 TIM_HandleTypeDef* htim_motor_2, uint32_t channel_motor_2, TIM_HandleTypeDef* htim_motor_3,
                 uint32_t channel_motor_3, TIM_HandleTypeDef* htim_motor_4, uint32_t channel_motor_4)
{
  htim_motor_1_ = htim_motor_1;
  channel_motor_1_ = channel_motor_1;
  htim_motor_2_ = htim_motor_2;
  channel_motor_2_ = channel_motor_2;
  htim_motor_3_ = htim_motor_3;
  channel_motor_3_ = channel_motor_3;
  htim_motor_4_ = htim_motor_4;
  channel_motor_4_ = channel_motor_4;

  dshot_set_timer(dshot_type);
  dshot_put_tc_callback_function();
  dshot_start_pwm();
}

void DShot::initTelemetry(UART_HandleTypeDef* huart)
{
  esc_reader_.init(huart);
  is_telemetry_ = true;
  esc_reader_.num_motor_mag_pole_ = 14; // TODO: should be set by onboard PC
}

void DShot::write(uint16_t* motor_value_array, bool is_telemetry)
{
  // Never rewrite a DMA buffer while a previous packet is still being transmitted.
  if (HAL_DMA_GetState(htim_motor_1_->hdma[TIM_DMA_ID_CC1]) != HAL_DMA_STATE_READY ||
      HAL_DMA_GetState(htim_motor_2_->hdma[TIM_DMA_ID_CC2]) != HAL_DMA_STATE_READY ||
      HAL_DMA_GetState(htim_motor_3_->hdma[TIM_DMA_ID_CC3]) != HAL_DMA_STATE_READY ||
      HAL_DMA_GetState(htim_motor_4_->hdma[TIM_DMA_ID_CC4]) != HAL_DMA_STATE_READY) return;

  bool is_telemetry_array[4] = {false, false, false, false};

  if (is_telemetry)
  {
    if (num_freq_divide == ESC_TELEM_REQUEST_DIVIDER - 1)
    {
      // receive the telemetry of the previous round
      if (id_telem_prev_ != -1)
      {
        switch (id_telem_prev_)
        {
          case 0:
            esc_reader_.update(esc_reader_.esc_msg_1_);
            break;
          case 1:
            esc_reader_.update(esc_reader_.esc_msg_2_);
            break;
          case 2:
            esc_reader_.update(esc_reader_.esc_msg_3_);
            break;
          case 3:
            esc_reader_.update(esc_reader_.esc_msg_4_);
            esc_reader_.is_update_all_msg_ = true;
            break;
        }
      }

      id_telem_prev_ = -1;
      // At the nominal 1 ms core period: one request per ESC every 5 ms.
      // Actual throughput and ESC turnaround time require hardware verification.
      if (id_telem_ >= 1 && id_telem_ <= 4)
      {
        esc_reader_.beginRequest(); // discard late/unassigned bytes before the next request
        id_telem_prev_ = id_telem_ - 1;
        is_telemetry_array[id_telem_prev_] = true;
      }
      id_telem_ = (id_telem_ + 1) % ESC_TELEM_SLOT_COUNT;

      num_freq_divide = 0;
    }
    else
    {
      num_freq_divide++;
    }
  }

  // send dshot signal
  dshot_prepare_dmabuffer_all(motor_value_array, is_telemetry_array);
  if (!dshot_dma_start())
  {
    // No request was put on the wire. Invalidate the round instead of assigning
    // later UART bytes to an ESC whose request could not be transmitted.
    id_telem_ = 0;
    id_telem_prev_ = -1;
    esc_reader_.is_update_all_msg_ = false;
    esc_reader_.esc_msg_1_.crc_error = esc_reader_.esc_msg_2_.crc_error = -1;
    esc_reader_.esc_msg_3_.crc_error = esc_reader_.esc_msg_4_.crc_error = -1;
    return;
  }
  dshot_enable_dma_request();
}

/* Static functions */
uint32_t DShot::dshot_choose_type(dshot_type_e dshot_type)
{
  switch (dshot_type)
  {
    case (DSHOT600):
      return DSHOT600_HZ;

    case (DSHOT300):
      return DSHOT300_HZ;

    case (DSHOT150):
      return DSHOT150_HZ;

    default:
      return DSHOT300_HZ;
  }
}

void DShot::dshot_set_timer(dshot_type_e dshot_type)
{
  uint16_t dshot_prescaler;
  uint32_t timer_clock = TIMER_CLOCK;  // all timer clock is same as SystemCoreClock in stm32f411

  // Calculate prescaler by dshot type
  dshot_prescaler = lrintf((float)timer_clock / dshot_choose_type(dshot_type) + 0.01f) - 1;

#if STM32H7_V2
  // Use TIM1 directly: 200 MHz / 333 ticks is approximately DShot600.
  // Keeping the legacy 20-tick divider here would give a much larger rate error.
  const uint32_t ticks = lrintf((float)timer_clock / (dshot_choose_type(dshot_type) / MOTOR_BITLENGTH));
  dshot_prescaler = 0;
  const uint32_t period = ticks - 1;
  motor_bit_0_ = lrintf(ticks * 0.375f);
  motor_bit_1_ = lrintf(ticks * 0.750f);
#else
  const uint32_t period = MOTOR_BITLENGTH;
#endif

  // motor1
  __HAL_TIM_SET_PRESCALER(htim_motor_1_, dshot_prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim_motor_1_, period);

  // motor2
  __HAL_TIM_SET_PRESCALER(htim_motor_2_, dshot_prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim_motor_2_, period);

  // motor3
  __HAL_TIM_SET_PRESCALER(htim_motor_3_, dshot_prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim_motor_3_, period);

  // motor4
  __HAL_TIM_SET_PRESCALER(htim_motor_4_, dshot_prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim_motor_4_, period);
}

// __HAL_TIM_DISABLE_DMA is needed to eliminate the delay between different dshot signals
// I don't know why :(
// After adding this function, there are no delay among dshot 1,2,3,4.
void DShot::dshot_dma_tc_callback(DMA_HandleTypeDef* hdma)
{
  TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  if (hdma == htim->hdma[TIM_DMA_ID_CC1])
  {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
  }
  else if (hdma == htim->hdma[TIM_DMA_ID_CC2])
  {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
  }
  else if (hdma == htim->hdma[TIM_DMA_ID_CC3])
  {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
  }
  else if (hdma == htim->hdma[TIM_DMA_ID_CC4])
  {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
  }
}

void DShot::dshot_put_tc_callback_function()
{
  // TIM_DMA_ID_CCx depends on timer channel
  htim_motor_1_->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
  htim_motor_2_->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshot_dma_tc_callback;
  htim_motor_3_->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = dshot_dma_tc_callback;
  htim_motor_4_->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshot_dma_tc_callback;
}

void DShot::dshot_start_pwm()
{
#if STM32H7_V2
  // Latch the new prescaler before enabling the outputs; initial CCRs are zero.
  htim_motor_1_->Instance->EGR = TIM_EGR_UG;
#endif
  // Start the timer channel now.
  // Enabling/disabling DMA request can restart a new cycle without PWM start/stop.
  HAL_TIM_PWM_Start(htim_motor_1_, channel_motor_1_);
  HAL_TIM_PWM_Start(htim_motor_2_, channel_motor_2_);
  HAL_TIM_PWM_Start(htim_motor_3_, channel_motor_3_);
  HAL_TIM_PWM_Start(htim_motor_4_, channel_motor_4_);
}

uint16_t DShot::dshot_prepare_packet(uint16_t value, bool dshot_telemetry)
{
  uint16_t packet;

  packet = (value << 1) | (dshot_telemetry ? 1 : 0);

  // compute checksum
  unsigned csum = 0;
  unsigned csum_data = packet;

  for (int i = 0; i < 3; i++)
  {
    csum ^= csum_data;  // xor data by nibbles
    csum_data >>= 4;
  }

  csum &= 0xf;
  packet = (packet << 4) | csum;

  return packet;
}

// Convert 16 bits packet to 16 pwm signal
void DShot::dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value, bool is_telemetry)
{
  uint16_t packet;
  packet = dshot_prepare_packet(value, is_telemetry);

  for (int i = 0; i < 16; i++)
  {
    motor_dmabuffer[i] = (packet & 0x8000) ? motor_bit_1_ : motor_bit_0_;
    packet <<= 1;
  }

  motor_dmabuffer[16] = 0;
  motor_dmabuffer[17] = 0;
}

void DShot::dshot_prepare_dmabuffer_all(uint16_t* motor_value, bool* is_telemetry)
{
  dshot_prepare_dmabuffer(motor1_dmabuffer_, motor_value[0], is_telemetry[0]);
  dshot_prepare_dmabuffer(motor2_dmabuffer_, motor_value[1], is_telemetry[1]);
  dshot_prepare_dmabuffer(motor3_dmabuffer_, motor_value[2], is_telemetry[2]);
  dshot_prepare_dmabuffer(motor4_dmabuffer_, motor_value[3], is_telemetry[3]);
}

bool DShot::dshot_dma_start()
{
  const HAL_StatusTypeDef status1 = HAL_DMA_Start_IT(htim_motor_1_->hdma[TIM_DMA_ID_CC1], (uint32_t)motor1_dmabuffer_,
                   (uint32_t)&htim_motor_1_->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
  const HAL_StatusTypeDef status2 = HAL_DMA_Start_IT(htim_motor_2_->hdma[TIM_DMA_ID_CC2], (uint32_t)motor2_dmabuffer_,
                   (uint32_t)&htim_motor_2_->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
  const HAL_StatusTypeDef status3 = HAL_DMA_Start_IT(htim_motor_3_->hdma[TIM_DMA_ID_CC3], (uint32_t)motor3_dmabuffer_,
                   (uint32_t)&htim_motor_3_->Instance->CCR3, DSHOT_DMA_BUFFER_SIZE);
  const HAL_StatusTypeDef status4 = HAL_DMA_Start_IT(htim_motor_4_->hdma[TIM_DMA_ID_CC4], (uint32_t)motor4_dmabuffer_,
                   (uint32_t)&htim_motor_4_->Instance->CCR4, DSHOT_DMA_BUFFER_SIZE);
  if (status1 == HAL_OK && status2 == HAL_OK && status3 == HAL_OK && status4 == HAL_OK) return true;

  // Requests are still disabled here: abort any channels that did start, and leave
  // every output low rather than enabling a partial/stale DMA transfer.
  HAL_DMA_Abort(htim_motor_1_->hdma[TIM_DMA_ID_CC1]);
  HAL_DMA_Abort(htim_motor_2_->hdma[TIM_DMA_ID_CC2]);
  HAL_DMA_Abort(htim_motor_3_->hdma[TIM_DMA_ID_CC3]);
  HAL_DMA_Abort(htim_motor_4_->hdma[TIM_DMA_ID_CC4]);
  __HAL_TIM_SET_COMPARE(htim_motor_1_, channel_motor_1_, 0);
  __HAL_TIM_SET_COMPARE(htim_motor_2_, channel_motor_2_, 0);
  __HAL_TIM_SET_COMPARE(htim_motor_3_, channel_motor_3_, 0);
  __HAL_TIM_SET_COMPARE(htim_motor_4_, channel_motor_4_, 0);
  return false;
}

void DShot::dshot_enable_dma_request()
{
  __HAL_TIM_ENABLE_DMA(htim_motor_1_, TIM_DMA_CC1);
  __HAL_TIM_ENABLE_DMA(htim_motor_2_, TIM_DMA_CC2);
  __HAL_TIM_ENABLE_DMA(htim_motor_3_, TIM_DMA_CC3);
  __HAL_TIM_ENABLE_DMA(htim_motor_4_, TIM_DMA_CC4);
}
