// Created by Chengfu Zou
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

#include "rm_serial_driver/protocol/infantry_protocol.hpp"
#include <iostream>

namespace fyt::serial_driver::protocol {
ProtocolInfantry::ProtocolInfantry(std::string_view port_name, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  packet_tool_ = std::make_shared<FixedPacketTool<19>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void ProtocolInfantry::send(const rm_interfaces::msg::GimbalCmd &data) {
  FixedPacket<19> packet;
  try {
  packet.loadData<unsigned char>(data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
  packet.loadData<float>(static_cast<float>(data.pitch), 2);
  packet.loadData<float>(static_cast<float>(data.yaw), 6);
  packet.loadData<float>(static_cast<float>(data.distance), 10);
  packet_tool_->sendPacket(packet);
  } catch (const std::exception& e) {
    std::cerr << "infantry_protocol: Error sending packet: " << e.what() << std::endl;
  }

  
}

bool ProtocolInfantry::receive(rm_interfaces::msg::SerialReceiveData &data) {
  FixedPacket<19> packet;
  try {
  if (packet_tool_->recvPacket(packet)) {
    packet.unloadData(data.mode, 1);
    packet.unloadData(data.bullet_speed, 2);
    packet.unloadData(data.roll, 6);
    packet.unloadData(data.pitch, 10);
    packet.unloadData(data.yaw, 14);
    return true;
  }
  } catch (const std::exception& e) {
    std::cerr << "infantry_protocol: Error receiving packet: " << e.what() << std::endl;
  }
    return false;
  
}

// 定义一个名为 ProtocolInfantry 的类中的成员函数 getSubscriptions
// 该函数接收一个指向 rclcpp::Node 的共享指针作为参数，并返回一个包含 rclcpp::SubscriptionBase 共享指针的向量
std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolInfantry::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  // 创建第一个订阅者，订阅主题 "armor_solver/cmd_gimbal"，消息类型为 rm_interfaces::msg::GimbalCmd
  // 使用 rclcpp::SensorDataQoS() 设置服务质量
  // 订阅回调函数为 lambda 表达式，接收一个 rm_interfaces::msg::GimbalCmd 消息的共享指针，并调用当前对象的 send 方法
  auto sub1 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  // 创建第二个订阅者，订阅主题 "rune_solver/cmd_gimbal"，消息类型为 rm_interfaces::msg::GimbalCmd
  // 使用 rclcpp::SensorDataQoS() 设置服务质量
  // 订阅回调函数为 lambda 表达式，接收一个 rm_interfaces::msg::GimbalCmd 消息的共享指针，并调用当前对象的 send 方法
  auto sub2 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  // 返回包含两个订阅者共享指针的向量
  return {sub1, sub2};
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolInfantry::getClients(
  rclcpp::Node::SharedPtr node) const {
  auto client1 = node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client2 = node->create_client<rm_interfaces::srv::SetMode>("armor_solver/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client3 = node->create_client<rm_interfaces::srv::SetMode>("rune_detector/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client4 = node->create_client<rm_interfaces::srv::SetMode>("rune_solver/set_mode",
                                                                  rmw_qos_profile_services_default);
  return {client1, client2, client3, client4};
}

// 实现获取 Transporter 的方法
std::shared_ptr<TransporterInterface> ProtocolInfantry::getTransporter() {
  return packet_tool_->getTransporter();
}

}  // namespace fyt::serial_driver::protocol
