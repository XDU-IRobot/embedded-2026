/*
  Copyright (c) 2026 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/**
 * @file  librm/device/referee/referee.hpp
 * @brief 裁判系统
 */

#ifndef LIBRM_DEVICE_REFEREE_USER_HPP
#define LIBRM_DEVICE_REFEREE_USER_HPP

#include "protocol_user.hpp"
#include <librm.hpp>

#include <cstring>

#include "librm/device/device.hpp"

namespace rm::device {
/**
 * @brief   裁判系统用户协议
 * @note    也可以把这个类当做一个通用的字节流通信分包器来用，
 *          只需要把通信协议按照设计实现出来即可（参考现有裁判系统协议实现 protocol_vXXX.hpp/.cc）
 */
template <RefereeRevision revision>
class RefereeUser final : public Device {
 private:
  RefereeSubProtocol deserialize_buffer_{};
  RefereeSubProtocolMemoryMap referee_protocol_memory_map_;
  Referee<revision> &referee_;

 public:
  RefereeUser() = delete;

  explicit RefereeUser(Referee<revision> &referee) : referee_(referee) {}

  const RefereeSubProtocol &data() const { return deserialize_buffer_; }

  // 增加 data_len_this_time_ 参数以便传入本次接收的数据长度
  void AttachCallback(u16 cmd_id_, u8 seq_) {
    ReportStatus(kOk);
    // 将裁判系统的数据拷贝到反序列化缓冲区。
    u16 subCmdID = referee_.data().robot_interaction_data.data_cmd_id;
    if (cmd_id_ != 0x301) return;
    if (!rm::device::RefereeSubProtocolMemoryMap::map.contains(subCmdID)) return;
    const usize member_offset = rm::device::RefereeSubProtocolMemoryMap::map.at(subCmdID);
    u8 *dest_ptr = reinterpret_cast<u8 *>(&deserialize_buffer_) + member_offset;
    u8 *src_ptr = const_cast<u8 *>(referee_.data().robot_interaction_data.user_data);
    std::memcpy(dest_ptr, src_ptr, rm::device::RefereeSubProtocolMemoryMap::mapSize.at(subCmdID));
  }
};
}  // namespace rm::device

namespace rm::device {
template <typename T>
[[nodiscard]] inline u8 Referee0x301Prepare(u8 *data, const u16 start_index, T &info, const u16 sender, const u16 receiver) {
  static u8 seq_ = 0;
  u16 index_ = start_index;
  data[index_++] = kRefProtocolHeaderSof;
  constexpr u16 data_len = sizeof(info) + 6;
  data[index_++] = data_len & 0xff;
  data[index_++] = data_len >> 8;
  data[index_++] = seq_++;
  data[index_++] = modules::Crc8(&data[start_index], kRefProtocolHeaderLen - 1, modules::CRC8_INIT);
  data[index_++] = 0x301 & 0xff;
  data[index_++] = 0x301 >> 8;
  data[index_++] = getCmd(info) & 0xff;
  data[index_++] = getCmd(info) >> 8;
  data[index_++] = sender & 0xff;
  data[index_++] = sender >> 8;
  data[index_++] = receiver & 0xff;
  data[index_++] = receiver >> 8;
  std::memcpy(&data[index_], &info, sizeof(info));
  index_ += sizeof(info);
  const u16 CRC16_ = modules::Crc16(&data[start_index], index_ - start_index, modules::CRC16_INIT);
  data[index_++] = CRC16_ & 0xff;
  data[index_++] = CRC16_ >> 8;
  return index_ - start_index;
};

inline u8 RefereePrepare(u8 *data, const u16 start_index, const struct MapRobotPosition &structInfo) {
  static u8 seq_ = 0;
  u16 index_ = start_index;
  data[index_++] = kRefProtocolHeaderSof;
  constexpr u16 data_len = sizeof(structInfo);
  data[index_++] = data_len & 0xff;
  data[index_++] = data_len >> 8;
  data[index_++] = seq_++;
  data[index_++] = modules::Crc8(&data[start_index], kRefProtocolHeaderLen - 1, modules::CRC8_INIT);
  data[index_++] = getCmd(structInfo) & 0xff;
  data[index_++] = getCmd(structInfo) >> 8;
  std::memcpy(&data[index_], &structInfo, sizeof(structInfo));
  index_ += sizeof(structInfo);
  const u16 CRC16_ = modules::Crc16(&data[start_index], index_ - start_index, modules::CRC16_INIT);
  data[index_++] = CRC16_ & 0xff;
  data[index_++] = CRC16_ >> 8;
  return index_ - start_index;
};
}  // namespace rm::device

#endif  // LIBRM_DEVICE_REFEREE_USER_HPP