// Created by Chengfu Zou on 2023.7.6
// Copyright (C) FYT Vision Group. All rights reserved.
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

#ifndef SERIAL_DRIVER_PROTOCOL_HPP_
#define SERIAL_DRIVER_PROTOCOL_HPP_

// std
#include <memory>
#include <string>
#include <string_view>
// ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_interfaces/msg/chassis_cmd.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/uart_transporter.hpp"

namespace fyt::serial_driver {
namespace protocol {
typedef enum : unsigned char { Fire = 0x01, NotFire = 0x00 } FireState;

// Protocol interface
class Protocol {
public:
  virtual ~Protocol() = default;

  // Send gimbal command
  virtual void send(const rm_interfaces::msg::GimbalCmd &data) = 0;

  // Receive data from serial port
  virtual bool receive(rm_interfaces::msg::SerialReceiveData &data) = 0;

  // Create subscriptions for SerialDriverNode
  virtual std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(
    rclcpp::Node::SharedPtr node) = 0;

  // Cretate setMode client for SerialDriverNode
  virtual std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node::SharedPtr node) const = 0;

  virtual std::string getErrorMessage() = 0;

  // 抽象基类，用于不同尺寸的FixedPacketTool
  class PacketToolBase {
  public:
    virtual ~PacketToolBase() = default;
    virtual void enableInputPrint(bool enable) = 0;
    virtual void enableOutputPrint(bool enable) = 0;
  };

  // 泛型包装类，用于包装不同尺寸的FixedPacketTool
  template <int capacity>
  class PacketToolWrapper : public PacketToolBase {
  public:
    explicit PacketToolWrapper(std::shared_ptr<FixedPacketTool<capacity>> tool) : tool_(tool) {}
    void enableInputPrint(bool enable) override { tool_->enableInputPrint(enable); }
    void enableOutputPrint(bool enable) override { tool_->enableOutputPrint(enable); }
  private:
    std::shared_ptr<FixedPacketTool<capacity>> tool_;
  };

  // 获取通信工具对象的方法，返回包装后的基类指针
  virtual std::shared_ptr<PacketToolBase> getPacketTool() { return nullptr; }

  // 获取底层的 Transporter 对象
  virtual std::shared_ptr<TransporterInterface> getTransporter() { return nullptr; }

private:
};

}  // namespace protocol
}  // namespace fyt::serial_driver
#endif  // SERIAL_DRIVER_PROTOCOLS_HPP_
