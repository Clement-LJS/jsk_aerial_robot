#ifndef JSK_DYNAMIXEL_PACKET_H
#define JSK_DYNAMIXEL_PACKET_H

#include <stddef.h>
#include <stdint.h>

// Protocol 2.0: CRC covers the stuffed packet, including its updated length.
namespace dynamixel_packet {
inline uint16_t crc(const uint8_t* data, size_t size)
{
  uint16_t value = 0;
  for (size_t i = 0; i < size; ++i) {
    value ^= static_cast<uint16_t>(data[i]) << 8;
    for (unsigned bit = 0; bit < 8; ++bit)
      value = (value & 0x8000) ? (value << 1) ^ 0x8005 : value << 1;
  }
  return value;
}

inline size_t encode(uint8_t id, uint8_t instruction, const uint8_t* parameters,
                     size_t count, uint8_t* out, size_t capacity)
{
  if (capacity < 10 || (count && !parameters)) return 0;
  out[0] = 0xff; out[1] = 0xff; out[2] = 0xfd; out[3] = 0;
  out[4] = id;
  size_t size = 7;
  for (size_t i = 0; i <= count; ++i) {
    if (size + 2 >= capacity) return 0;
    out[size++] = i ? parameters[i - 1] : instruction;
    if (size >= 10 && out[size - 3] == 0xff && out[size - 2] == 0xff && out[size - 1] == 0xfd) {
      if (size + 2 >= capacity) return 0;
      out[size++] = 0xfd;
    }
  }
  const size_t length = size - 7 + 2;
  out[5] = length & 0xff; out[6] = length >> 8;
  const uint16_t checksum = crc(out, size);
  out[size++] = checksum & 0xff; out[size++] = checksum >> 8;
  return size;
}

// Decode the complete instruction/error/parameter body only after CRC validation.
inline size_t decodeStatus(const uint8_t* packet, size_t size, uint8_t* body, size_t capacity)
{
  if (size < 11 || packet[0] != 0xff || packet[1] != 0xff ||
      packet[2] != 0xfd || packet[3] != 0 || packet[4] >= 0xfd ||
      size != 7u + packet[5] + (static_cast<size_t>(packet[6]) << 8) ||
      crc(packet, size - 2) != (packet[size - 2] | (packet[size - 1] << 8))) return 0;
  size_t count = 0;
  for (size_t i = 7; i < size - 2; ++i) {
    if (count == capacity) return 0;
    body[count++] = packet[i];
    if (i >= 9 && packet[i - 2] == 0xff && packet[i - 1] == 0xff && packet[i] == 0xfd) {
      if (++i >= size - 2 || packet[i] != 0xfd) return 0;
    }
  }
  return count >= 2 && body[0] == 0x55 && !(body[1] & 0x7f) ? count : 0;
}

inline int32_t signed32(const uint8_t* data)
{
  const uint32_t value = static_cast<uint32_t>(data[0]) |
      (static_cast<uint32_t>(data[1]) << 8) | (static_cast<uint32_t>(data[2]) << 16) |
      (static_cast<uint32_t>(data[3]) << 24);
  return static_cast<int32_t>(value);
}
inline int32_t singleTurn(int32_t value) { return (value % 4096 + 4096) % 4096; }
}
#endif
