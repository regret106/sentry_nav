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

#include "rm_serial_driver/protocol/sentry_protocol.hpp"
// ros2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
// 添加用于格式化输出的头文件
#include <cmath>
#include <iomanip>
#include <iostream>

namespace fyt::serial_driver::protocol {
namespace {

double getYaw(const geometry_msgs::msg::Quaternion & quaternion) {
  const double siny_cosp =
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp =
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

ProtocolSentry::ProtocolSentry(std::string_view port_name, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  packet_tool_ = std::make_shared<FixedPacketTool<32>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void ProtocolSentry::send(const rm_interfaces::msg::GimbalCmd &data) {
  packet_.loadData<unsigned char>(data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
  // is_spin
  packet_.loadData<unsigned char>(data.is_spining ? 0x01 : 0x00, 2);
  packet_.loadData<unsigned char>(data.is_navigating ? 0x01 : 0x00, 3);
  // packet_.loadData<unsigned char>(0x00, 2);
  // gimbal control
  packet_.loadData<float>(static_cast<float>(data.pitch), 4);
  packet_.loadData<float>(static_cast<float>(data.yaw), 8);
  packet_.loadData<float>(static_cast<float>(data.distance), 12);
  // // chassis control
  // linear x
  packet_.loadData<float>(data.twist.linear.x, 16);
  // linear y
  packet_.loadData<float>(data.twist.linear.y, 20);
  // angular z
  packet_.loadData<float>(data.twist.angular.z, 24);
  // // useless data
  // packet_.loadData<float>(0, 28);
  packet_tool_->sendPacket(packet_);
}

void ProtocolSentry::send(const rm_interfaces::msg::ChassisCmd &data) {
  // 添加详细的调试信息 - 打印输入的ChassisCmd数据
  std::cout << "serial_driver: 发送ChassisCmd: is_spining=" << (data.is_spining ? "true" : "false")
            << ", is_navigating=" << (data.is_navigating ? "true" : "false")
            << ", twist=[" << std::fixed << std::setprecision(3) << data.twist.linear.x 
            << ", " << data.twist.linear.y 
            << ", " << data.twist.angular.z << "]" << std::endl;

  // packet_.loadData<unsigned char>(0x00, 1);
  // is_spin
  packet_.loadData<unsigned char>(data.is_spining ? 0x01 : 0x00, 2);
  packet_.loadData<unsigned char>(data.is_navigating ? 0x01 : 0x00, 3);
  // gimbal control
  // packet_.loadData<float>(0, 4);
  // packet_.loadData<float>(0, 8);
  // packet_.loadData<float>(0, 12);
  // chassis control
  // linear x
  packet_.loadData<float>(data.twist.linear.x, 16);
  // linear y
  packet_.loadData<float>(data.twist.linear.y, 20);
  // angular z
  packet_.loadData<float>(data.twist.angular.z, 24);
  // useless data
  // packet_.loadData<float>(0, 28);
  

  
  packet_tool_->sendPacket(packet_);
}

void ProtocolSentry::sendGoalPose(const geometry_msgs::msg::PoseStamped & data) {
  rm_interfaces::msg::ChassisCmd chassis_cmd;
  chassis_cmd.is_spining = false;
  chassis_cmd.is_navigating = true;
  chassis_cmd.twist.linear.x = static_cast<float>(data.pose.position.x);
  chassis_cmd.twist.linear.y = static_cast<float>(data.pose.position.y);
  chassis_cmd.twist.angular.z = static_cast<float>(getYaw(data.pose.orientation));

  RCLCPP_INFO(
    rclcpp::get_logger("sentry_protocol"),
    "Forwarding preset goal pose to lower controller: x=%.3f, y=%.3f, yaw=%.3f",
    chassis_cmd.twist.linear.x, chassis_cmd.twist.linear.y, chassis_cmd.twist.angular.z);

  send(chassis_cmd);
}

bool ProtocolSentry::receive(rm_interfaces::msg::SerialReceiveData &data) {
  FixedPacket<32> packet;
  
  if (packet_tool_->recvPacket(packet)) {
    uint8_t enemy_color;
    packet.unloadData(enemy_color, 1);
    data.mode = (enemy_color == ENEMY_BLUE ? 1 : 0);

    packet.unloadData(data.pitch, 2);
    packet.unloadData(data.yaw, 6);
    
    packet.unloadData(data.judge_system_data.blood, 14);
    packet.unloadData(data.judge_system_data.remaining_time, 16);
    packet.unloadData(data.judge_system_data.outpost_hp, 20);

    packet.unloadData(data.judge_system_data.operator_command.is_outpost_attacking, 22);
    packet.unloadData(data.judge_system_data.operator_command.is_retreating, 23);
    packet.unloadData(data.judge_system_data.operator_command.is_drone_avoiding, 24);

    packet.unloadData(data.judge_system_data.game_status, 25);

    data.bullet_speed = 25;
    return true;
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("sentry_protocol"), 
                "Failed to receive packet: %s", 
                packet_tool_->getErrorMessage().c_str());
    return false;
  }
}

// 实现获取 Transporter 的方法
std::shared_ptr<TransporterInterface> ProtocolSentry::getTransporter() {
  return packet_tool_->getTransporter();
}

// 定义ProtocolSentry类的成员函数getSubscriptions，该函数接收一个rclcpp::Node::SharedPtr类型的参数node
std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolSentry::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  // 创建一个订阅者sub1，订阅主题"armor_solver/cmd_gimbal"，消息类型为rm_interfaces::msg::GimbalCmd
  auto sub1 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  
  // 创建一个订阅者sub2，订阅主题"rune_solver/cmd_gimbal"，消息类型为rm_interfaces::msg::GimbalCmd
  auto sub2 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  
  RCLCPP_INFO(rclcpp::get_logger("sentry_protocol"), "create subscription for /cmd_vel");
  // 创建一个订阅者sub3，订阅主题"/cmd_vel"，消息类型为geometry_msgs::msg::Twist
  auto sub3 = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      // 将Twist消息转换为ChassisCmd
      rm_interfaces::msg::ChassisCmd chassis_cmd;
      
      // 设置默认参数
      chassis_cmd.is_spining = false;  // 默认不自旋
      chassis_cmd.is_navigating = true;  // 默认在导航模式
      // 复制速度信息
      chassis_cmd.twist = *msg;
      // TODO 暂时关闭自旋
      chassis_cmd.twist.angular.z = 0;
      // 打印调试信息 - 将级别改为 INFO
      RCLCPP_INFO(rclcpp::get_logger("sentry_protocol"), 
                  "Converting cmd_vel to ChassisCmd: linear=[%.2f, %.2f, %.2f], angular=[%.2f, %.2f, %.2f]",
                  msg->linear.x, msg->linear.y, msg->linear.z,
                  msg->angular.x, msg->angular.y, msg->angular.z);
      
      // 发送转换后的消息
      this->send(chassis_cmd);
    });

  auto sub4 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->sendGoalPose(*msg); });
  
  // 返回包含sub1, sub2, sub3, sub4的订阅者列表
  return {sub1, sub2, sub3, sub4};
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolSentry::getClients(
  rclcpp::Node::SharedPtr node) const {
  auto client1 = node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client2 = node->create_client<rm_interfaces::srv::SetMode>("armor_solver/set_mode",
                                                                  rmw_qos_profile_services_default);
  return {client1, client2};
}
}  // namespace fyt::serial_driver::protocol
