// KISS telemetry: isolated 10-byte UART bursts, polled at the next DShot slot.
#include "esc_telem.h"

namespace
{
#ifdef STM32H7
  uint8_t esc_telem_rx_buf_[ESC_BUFFER_SIZE] __attribute__((section(".EscRxBufferSection")));
#else
  uint8_t esc_telem_rx_buf_[ESC_BUFFER_SIZE];
#endif
  uint32_t esc_telem_rd_ptr_ = 0;
}

void ESCReader::init(UART_HandleTypeDef* huart)
{
  huart_ = huart;
  esc_msg_1_.crc_error = esc_msg_2_.crc_error = -1;
  esc_msg_3_.crc_error = esc_msg_4_.crc_error = -1;
  memset(esc_telem_rx_buf_, 0, ESC_BUFFER_SIZE);
  esc_telem_rd_ptr_ = 0;
  HAL_UART_Receive_DMA(huart_, esc_telem_rx_buf_, ESC_BUFFER_SIZE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
  __HAL_UART_CLEAR_IDLEFLAG(huart_);
}

void ESCReader::beginRequest()
{
  esc_telem_rd_ptr_ = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % ESC_BUFFER_SIZE;
  __HAL_UART_CLEAR_IDLEFLAG(huart_);
}

void ESCReader::update(spinal::ESCTelemetry& esc_msg)
{
  // The existing uint8 crc_error schema encodes the -1 dropped-frame sentinel as 255.
  // Mark missing/incomplete replies invalid too, so old voltage is never a new sample.
  esc_msg.crc_error = -1;
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE))
  {
    HAL_UART_AbortReceive(huart_);
    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);
    __HAL_UART_CLEAR_NEFLAG(huart_);
    memset(esc_telem_rx_buf_, 0, ESC_BUFFER_SIZE);
    esc_telem_rd_ptr_ = 0;
    HAL_UART_Receive_DMA(huart_, esc_telem_rx_buf_, ESC_BUFFER_SIZE);
    __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
    __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    return;
  }

  if (!__HAL_UART_GET_FLAG(huart_, UART_FLAG_IDLE))
  {
    beginRequest(); // discard a truncated burst; never splice it into another reply
    return;
  }
  __HAL_UART_CLEAR_IDLEFLAG(huart_);
  const uint32_t wr = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % ESC_BUFFER_SIZE;
  const uint32_t count = (wr + ESC_BUFFER_SIZE - esc_telem_rd_ptr_) % ESC_BUFFER_SIZE;
  if (count != 10)
  {
    esc_telem_rd_ptr_ = wr;
    return;
  }

  uint8_t buffer[10];
  for (unsigned int i = 0; i < 10; ++i)
    buffer[i] = esc_telem_rx_buf_[(esc_telem_rd_ptr_ + i) % ESC_BUFFER_SIZE];
  esc_telem_rd_ptr_ = wr;
  const uint8_t crc = get_crc8(buffer, 9);
  esc_msg.crc_error = crc - buffer[9];
  if (crc != buffer[9]) return;
  if (num_motor_mag_pole_ < 2) { esc_msg.crc_error = -1; return; }

  esc_msg.temperature = buffer[0];
  esc_msg.voltage = buffer[1] << 8 | buffer[2];
  esc_msg.current = buffer[3] << 8 | buffer[4];
  esc_msg.consumption = buffer[5] << 8 | buffer[6];
  const uint16_t erpm = buffer[7] << 8 | buffer[8];
  esc_msg.rpm = erpm * 100 / (num_motor_mag_pole_ / 2);
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}
