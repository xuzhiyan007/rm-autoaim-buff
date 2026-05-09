// Copyright 2025 JunLin

#ifndef jlcv_serial_driver__PACKET_HPP_
#define jlcv_serial_driver__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace jlcv_serial_driver
{
enum
{
  // 0x5A
  RECEVIDE_HEADER = 0x5A,
  // 0xA5
  SEND_HEADER_ANGLE = 0xA5,
};

struct ReceivePacket
{
  uint8_t header = RECEVIDE_HEADER;
  uint8_t enemy_color : 1;  // 敌方颜色  0-red 1-blue
  uint8_t game_status : 3;  // 比赛状态
  uint8_t outpost_status : 1;  // 前哨站状态
  uint8_t mode;  //SMALL_ENERGY:0  BIG_ENERGY:1  ARMOR:2
  float curr_pitch;
  float curr_yaw;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacketANGLE
{
  // geometry_msgs/msg/Vector3
  float pitch; //pitch
  float yaw; //yaw
  uint8_t shoot_flag; //shoot_flag 0:indistinction 1:tracking  2:fire
} __attribute__((packed));

struct SendPacket
{
  uint8_t header;
  union Data
  {
    struct SendPacketANGLE packetangle;
  } data;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t>& data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t*>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T& data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(reinterpret_cast<const uint8_t*>(&data), reinterpret_cast<const uint8_t*>(&data) + sizeof(T),
            packet.begin());
  return packet;
}

}  // namespace jlcv_serial_driver

#endif  // jlcv_serial_driver__PACKET_HPP_
