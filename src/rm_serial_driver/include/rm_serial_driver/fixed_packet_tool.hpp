// Copyright (C) 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Additional modifications and features by Chengfu Zou, 2023.
//
// Copyright (C) FYT Vision Group. All rights reserved.

#ifndef SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_
#define SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_

// std
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <iomanip>
#include <sstream>
// project
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace fyt::serial_driver {

// 将数据缓冲区转换为十六进制字符串用于调试
inline std::string bytesToHexString(const uint8_t* buffer, int len) {
  std::stringstream ss;
  for (int i = 0; i < len; ++i) {
    ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(buffer[i]) << " ";
    // 每8个字节添加一个空格增加可读性
    if ((i + 1) % 8 == 0) ss << " ";
  }
  return ss.str();
}

template <int capacity = 32>
class FixedPacketTool {
public:
  using SharedPtr = std::shared_ptr<FixedPacketTool>;
  FixedPacketTool() = delete;
  explicit FixedPacketTool(std::shared_ptr<TransporterInterface> transporter)
  : transporter_(transporter) {
    if (!transporter) {
      throw std::invalid_argument("transporter is nullptr");
    }
  }

  ~FixedPacketTool() { enbaleRealtimeSend(false); }

  bool isOpen() { return transporter_->isOpen(); }
  void enbaleRealtimeSend(bool enable);
  // 控制数据打印的方法
  void enbaleDataPrint(bool enable) { 
    use_data_print_ = enable; 
    std::cout << "Data printing is now " << (enable ? "enabled" : "disabled") << std::endl;
  }
  
  // 单独控制输入和输出打印的方法
  void enableInputPrint(bool enable) { 
    use_input_print_ = enable; 
    std::cout << "Input data printing is now " << (enable ? "enabled" : "disabled") << std::endl;
  }
  
  void enableOutputPrint(bool enable) { 
    use_output_print_ = enable; 
    std::cout << "Output data printing is now " << (enable ? "enabled" : "disabled") << std::endl;
  }

  bool sendPacket(const FixedPacket<capacity> &packet);
  bool recvPacket(FixedPacket<capacity> &packet);

  std::string getErrorMessage() {  std::string error_msg = transporter_->errorMessage();
  if (error_msg.empty()) {
    error_msg = "Unknown error occurred";
  }
  return error_msg;}

  // 获取transporter_成员
  std::shared_ptr<TransporterInterface> getTransporter() { return transporter_; }

private:
  bool checkPacket(uint8_t *buffer, int recv_len);
  bool simpleSendPacket(const FixedPacket<capacity> &packet);

private:
  std::shared_ptr<TransporterInterface> transporter_;
  // data
  uint8_t tmp_buffer_[capacity];       // NOLINT
  uint8_t recv_buffer_[capacity * 2];  // NOLINT
  int recv_buf_len_;
  // for realtime sending
  bool use_realtime_send_{false};
  bool use_data_print_{false};       // 控制所有数据打印
  bool use_input_print_{false};      // 控制输入数据打印，默认关闭
  bool use_output_print_{true};      // 控制输出数据打印，默认开启
  std::mutex realtime_send_mut_;
  std::unique_ptr<std::thread> realtime_send_thread_;
  std::queue<FixedPacket<capacity>> realtime_packets_;
};

template <int capacity>
bool FixedPacketTool<capacity>::checkPacket(uint8_t *buffer, int recv_len) {
  // 添加更详细的调试信息
  if (recv_len != capacity) {
    RCLCPP_DEBUG(rclcpp::get_logger("fixed_packet_tool"), 
                "数据包长度不匹配: 期望 %d, 实际 %d", 
                capacity, recv_len);
    return false;
  }

  // 检查帧头
  if (buffer[0] != 0xff) {
    RCLCPP_DEBUG(rclcpp::get_logger("fixed_packet_tool"), 
                "帧头无效: 期望 0xff, 实际 0x%02x, 完整数据: %s", 
                static_cast<int>(buffer[0]),
                bytesToHexString(buffer, recv_len).c_str());
    return false;
  }

  // 检查帧尾
  if (buffer[capacity - 1] != 0x0d) {
    RCLCPP_DEBUG(rclcpp::get_logger("fixed_packet_tool"), 
                "帧尾无效: 期望 0x0d, 实际 0x%02x, 完整数据: %s", 
                static_cast<int>(buffer[capacity - 1]),
                bytesToHexString(buffer, recv_len).c_str());
    return false;
  }

  return true;
}

template <int capacity>
bool FixedPacketTool<capacity>::simpleSendPacket(const FixedPacket<capacity> &packet) {
  try {
    // 打印输出数据
    if (use_data_print_ || use_output_print_) {
      std::stringstream ss;
      ss << "Sending data [" << capacity << " bytes]: " << bytesToHexString(packet.buffer(), capacity);
      std::cout << ss.str() << std::endl;
    }
    
    if (transporter_->write(packet.buffer(), capacity) == capacity) {
      return true;
    } else {
      std::cerr << "Transporter write failed" << std::endl;
      transporter_->close();
      transporter_->open();
      return false;
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception during write: " << e.what() << std::endl;
    return false;
  }
}

template <int capacity>
void FixedPacketTool<capacity>::enbaleRealtimeSend(bool enable) {
  if (enable == use_realtime_send_) {
    return;
  }
  if (enable) {
    use_realtime_send_ = true;
    realtime_send_thread_ = std::make_unique<std::thread>([&]() {
      FixedPacket<capacity> packet;
      while (use_realtime_send_) {
        bool empty = true;
        {
          std::lock_guard<std::mutex> lock(realtime_send_mut_);
          empty = realtime_packets_.empty();
          if (!empty) {
            packet = realtime_packets_.front();
            realtime_packets_.pop();
          }
        }
        if (!empty) {
          simpleSendPacket(packet);
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
    });
  } else {
    use_realtime_send_ = false;
    realtime_send_thread_->join();
    realtime_send_thread_.reset();
  }
}

template <int capacity>
bool FixedPacketTool<capacity>::sendPacket(const FixedPacket<capacity> &packet) {
  if (use_realtime_send_) {
    std::lock_guard<std::mutex> lock(realtime_send_mut_);
    realtime_packets_.push(packet);
    return true;
  } else {
    return simpleSendPacket(packet);
  }
}

// 模板类 FixedPacketTool 的成员函数 recvPacket 的定义
// 该函数用于接收固定大小的数据包
template<int capacity>
bool FixedPacketTool<capacity>::recvPacket(FixedPacket<capacity> &packet) {
  // 从传输器读取数据到临时缓冲区
  int recv_len = transporter_->read(tmp_buffer_, capacity);
  
  if (recv_len > 0) {
    // 打印接收数据（仅当启用时）
    if (use_data_print_ || use_input_print_) {
      std::stringstream ss;
      ss << "接收数据 [" << recv_len << " bytes]: " << bytesToHexString(tmp_buffer_, recv_len);
      RCLCPP_DEBUG(rclcpp::get_logger("fixed_packet_tool"), "%s", ss.str().c_str());
    }

    // 检查数据包
    if (checkPacket(tmp_buffer_, recv_len)) {
      packet.copyFrom(tmp_buffer_);
      return true;
    } else {
      // 断帧处理逻辑优化
      if (recv_buf_len_ + recv_len > capacity * 2) {
        RCLCPP_DEBUG(rclcpp::get_logger("fixed_packet_tool"), 
                    "缓冲区溢出，清空缓存: %d + %d > %d", 
                    recv_buf_len_, recv_len, capacity * 2);
        recv_buf_len_ = 0;
      }
      
      // 拼接缓存
      memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer_, recv_len);
      recv_buf_len_ += recv_len;
      
      RCLCPP_DEBUG(rclcpp::get_logger("fixed_packet_tool"), 
                  "尝试断帧重组: 当前缓存长度 %d, 新数据长度 %d", 
                  recv_buf_len_ - recv_len, recv_len);

      // 遍历校验
      for (int i = 0; (i + capacity) <= recv_buf_len_; i++) {
        if (checkPacket(recv_buffer_ + i, capacity)) {
          packet.copyFrom(recv_buffer_ + i);
          // 更新接收缓存
          int remaining_bytes = recv_buf_len_ - (i + capacity);
          if (remaining_bytes > 0) {
            memmove(recv_buffer_, recv_buffer_ + i + capacity, remaining_bytes);
          }
          recv_buf_len_ = remaining_bytes;
          
          RCLCPP_DEBUG(rclcpp::get_logger("fixed_packet_tool"), 
                      "断帧重组成功: 偏移量 %d, 剩余字节数 %d", 
                      i, remaining_bytes);
          return true;
        }
      }
      
      RCLCPP_DEBUG(rclcpp::get_logger("fixed_packet_tool"), 
                  "断帧重组失败: 缓存数据 %s", 
                  bytesToHexString(recv_buffer_, recv_buf_len_).c_str());
      return false;
    }
  } else {
    if (recv_len < 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("fixed_packet_tool"), 
                  "读取错误: %s", 
                  transporter_->errorMessage().c_str());
    }
    return false;
  }
}

using FixedPacketTool16 = FixedPacketTool<16>;
using FixedPacketTool32 = FixedPacketTool<32>;
using FixedPacketTool64 = FixedPacketTool<64>;

}  // namespace fyt::serial_driver

#endif  // SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_
