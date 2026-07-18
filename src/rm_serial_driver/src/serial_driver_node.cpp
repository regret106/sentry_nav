// Created by Chengfu Zou on 2023.7.1
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

#include "rm_serial_driver/serial_driver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
// std
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
// ros2
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_serial_driver/uart_transporter.hpp"

namespace fyt::serial_driver {
SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions &options)
: Node("serial_driver", options) {
  // Task thread
  listen_thread_ = std::make_unique<std::thread>(&SerialDriverNode::listenLoop, this);
}

void SerialDriverNode::init() {
  RCLCPP_INFO(get_logger(), "Initializing SerialDriverNode!");
  // Init
  target_frame_ = this->declare_parameter("target_frame", "odom");
  std::string port_name = this->declare_parameter("port_name", "/dev/ttyACM0");
  std::string protocol_type = this->declare_parameter("protocol", "sentry");
  bool enable_data_print = this->declare_parameter("enable_data_print", false);  // 关闭总体数据打印
  bool enable_input_print = this->declare_parameter("enable_input_print", false);  // 关闭输入数据打印
  bool enable_output_print = this->declare_parameter("enable_output_print", true);  // 保持输出数据打印开启
  
  // Create Protocol
  protocol_ = ProtocolFactory::createProtocol(protocol_type, port_name, enable_data_print);
  if (protocol_ == nullptr) {
    RCLCPP_FATAL(get_logger(), "Failed to create protocol with type: %s", protocol_type.c_str());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(get_logger(), "Protocol has been created with type: %s, port: %s", 
              protocol_type.c_str(), port_name.c_str());

  // 设置更细粒度的打印控制
  auto packet_tool = protocol_->getPacketTool();
  if (packet_tool) {
    packet_tool->enableInputPrint(enable_input_print);
    packet_tool->enableOutputPrint(enable_output_print);
    RCLCPP_INFO(get_logger(), "Configure print options: input=%s, output=%s", 
                enable_input_print ? "enabled" : "disabled", 
                enable_output_print ? "enabled" : "disabled");
  }

  // 尝试打开串口
  auto transporter = protocol_->getTransporter();
  if (transporter) {
    auto uart_transporter = std::dynamic_pointer_cast<UartTransporter>(transporter);
    if (uart_transporter) {
      if (!uart_transporter->open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open UART device: %s", uart_transporter->errorMessage().c_str());
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(get_logger(), "UART device opened successfully");
    } else {
      RCLCPP_WARN(get_logger(), "Transporter is not a UartTransporter, skipping open().");
    }
  } else {
    RCLCPP_WARN(get_logger(), "Protocol does not provide a transporter, skipping open().");
  }

  // Subscriptions
  subscriptions_ = protocol_->getSubscriptions(this->shared_from_this());
  for (auto sub : subscriptions_) {
    RCLCPP_INFO(get_logger(), "Subscribe to topic: %s", sub->get_topic_name());
  }
  // Publisher
  serial_receive_data_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
    "serial/receive", rclcpp::SensorDataQoS());

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Param client
  for (auto client : protocol_->getClients(this->shared_from_this())) {
    std::string name = client->get_service_name();
    set_mode_clients_.emplace(name, client);
    RCLCPP_INFO(get_logger(), "Create client for service: %s", name.c_str());
  }

  RCLCPP_INFO(get_logger(), "SerialDriverNode has been initialized!");
}

SerialDriverNode::~SerialDriverNode() {
  RCLCPP_INFO(get_logger(), "Destroy SerialDriverNode!");
  rclcpp::shutdown();
  if (listen_thread_ != nullptr) {
    listen_thread_->join();
  }
}

void SerialDriverNode::listenLoop() {
  if (protocol_ == nullptr) {
    // Lazy init because shared_from_this() is not available in constructor
    init();
  }

  rm_interfaces::msg::SerialReceiveData receive_data;
  while (rclcpp::ok()) {
    if (protocol_->receive(receive_data)) {
      receive_data.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      receive_data.header.frame_id = target_frame_;
      serial_receive_data_pub_->publish(receive_data);

      for (auto &[service_name, client] : set_mode_clients_) {
        if (client.mode.load() != receive_data.mode && !client.on_waiting.load()) {
          setMode(client, receive_data.mode);
        }
      }

      geometry_msgs::msg::TransformStamped t;
      timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
      t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      t.header.frame_id = target_frame_;
      t.child_frame_id = "gimbal_link";
      auto roll = receive_data.roll * M_PI / 180.0;
      auto pitch = -receive_data.pitch * M_PI / 180.0;
      auto yaw = receive_data.yaw * M_PI / 180.0;
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      t.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(t);

      // odom_rectify: 转了roll角后的坐标系
      Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
      Eigen::Vector3d rpy;
      Eigen::Matrix3d rotationMatrix = q_eigen.toRotationMatrix();
      rpy.x() = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
      rpy.y() = atan2(-rotationMatrix(2, 0), sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + 
                                                 rotationMatrix(2, 2) * rotationMatrix(2, 2)));
      rpy.z() = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
      
      q.setRPY(rpy.x(), 0, 0);
      t.header.frame_id = target_frame_;
      t.child_frame_id = target_frame_ + "_rectify";
      tf_broadcaster_->sendTransform(t);
    } else {
      auto error_message = protocol_->getErrorMessage();
      error_message = error_message.empty() ? "unknown" : error_message;
      // RCLCPP_WARN(get_logger(), "Failed to reveive packet! error message: %s", error_message.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

void SerialDriverNode::setMode(SetModeClient &client, const uint8_t mode) {
  using namespace std::chrono_literals;

  std::string service_name = client.ptr->get_service_name();
  // Wait for service
  while (!client.ptr->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service %s. Exiting.", 
                   service_name.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...", service_name.c_str());
  }
  if (!client.ptr->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service: %s is not available!", service_name.c_str());
    return;
  }
  // Send request
  auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
  req->mode = mode;

  client.on_waiting.store(true);
  auto result = client.ptr->async_send_request(
    req, [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
      client.on_waiting.store(false);
      if (result.get()->success) {
        client.mode.store(mode);
      }
    });
}

}  // namespace fyt::serial_driver

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::serial_driver::SerialDriverNode)
