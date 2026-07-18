#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"

namespace fyt::serial_driver {

class SentryRefereeBridgeNode : public rclcpp::Node {
public:
  explicit SentryRefereeBridgeNode(const rclcpp::NodeOptions &options)
  : Node("sentry_referee_bridge", options) {
    robot_id_ = this->declare_parameter<int>("robot_id", 7);
    max_hp_ = this->declare_parameter<int>("max_hp", 600);
    max_heat_ = this->declare_parameter<int>("max_heat", 400);
    max_ammo_ = this->declare_parameter<int>("max_ammo", 600);
    default_game_progress_ =
      this->declare_parameter<int>("default_game_progress", pb_rm_interfaces::msg::GameStatus::RUNNING);
    publish_empty_topics_ = this->declare_parameter<bool>("publish_empty_topics", true);

    game_status_pub_ =
      this->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
    robot_status_pub_ =
      this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
    hp_pub_ =
      this->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", 10);
    rfid_pub_ =
      this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
    event_pub_ =
      this->create_publisher<pb_rm_interfaces::msg::EventData>("referee/event_data", 10);
    ground_robot_pub_ = this->create_publisher<pb_rm_interfaces::msg::GroundRobotPosition>(
      "referee/ground_robot_position", 10);
    buff_pub_ = this->create_publisher<pb_rm_interfaces::msg::Buff>("referee/buff", 10);

    serial_sub_ = this->create_subscription<rm_interfaces::msg::SerialReceiveData>(
      "serial/receive", rclcpp::SensorDataQoS(),
      std::bind(&SentryRefereeBridgeNode::serialCallback, this, std::placeholders::_1));

    if (publish_empty_topics_) {
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SentryRefereeBridgeNode::publishFallbackTopics, this));
    }
  }

private:
  void serialCallback(const rm_interfaces::msg::SerialReceiveData::SharedPtr msg) {
    const auto stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
                         ? this->now()
                         : rclcpp::Time(msg->header.stamp);

    pb_rm_interfaces::msg::GameStatus game_status;
    game_status.game_progress = normalizeGameProgress(msg->judge_system_data.game_status);
    game_status.stage_remain_time = std::max<int>(msg->judge_system_data.remaining_time, 0);
    game_status_pub_->publish(game_status);

    pb_rm_interfaces::msg::RobotStatus robot_status;
    robot_status.robot_id = static_cast<uint8_t>(robot_id_);
    robot_status.robot_level = 1;
    robot_status.maximum_hp = static_cast<uint16_t>(max_hp_);
    robot_status.current_hp =
      static_cast<uint16_t>(std::clamp<int>(msg->judge_system_data.blood, 0, max_hp_));
    robot_status.shooter_barrel_cooling_value = 0;
    robot_status.shooter_barrel_heat_limit = static_cast<uint16_t>(max_heat_);
    robot_status.shooter_17mm_1_barrel_heat = 0;
    robot_status.robot_pos.orientation.w = 1.0;
    robot_status.armor_id = 0;
    robot_status.hp_deduction_reason = pb_rm_interfaces::msg::RobotStatus::ARMOR_HIT;
    robot_status.projectile_allowance_17mm = static_cast<uint16_t>(max_ammo_);
    robot_status.remaining_gold_coin = 0;
    robot_status.is_hp_deduced = last_hp_ >= 0 && msg->judge_system_data.blood < last_hp_;
    robot_status_pub_->publish(robot_status);

    pb_rm_interfaces::msg::GameRobotHP all_hp;
    assignSentryHp(all_hp, msg->mode, robot_status.current_hp);
    hp_pub_->publish(all_hp);

    pb_rm_interfaces::msg::RfidStatus rfid_status;
    rfid_pub_->publish(rfid_status);

    pb_rm_interfaces::msg::EventData event_data;
    event_pub_->publish(event_data);

    pb_rm_interfaces::msg::GroundRobotPosition ground_robot_position;
    ground_robot_pub_->publish(ground_robot_position);

    pb_rm_interfaces::msg::Buff buff;
    buff_pub_->publish(buff);

    last_hp_ = msg->judge_system_data.blood;
    last_msg_time_ = stamp;
    seen_serial_ = true;
  }

  void publishFallbackTopics() {
    if (seen_serial_) {
      return;
    }

    pb_rm_interfaces::msg::GameStatus game_status;
    game_status.game_progress = static_cast<uint8_t>(default_game_progress_);
    game_status.stage_remain_time = 420;
    game_status_pub_->publish(game_status);

    pb_rm_interfaces::msg::RobotStatus robot_status;
    robot_status.robot_id = static_cast<uint8_t>(robot_id_);
    robot_status.robot_level = 1;
    robot_status.current_hp = static_cast<uint16_t>(max_hp_);
    robot_status.maximum_hp = static_cast<uint16_t>(max_hp_);
    robot_status.shooter_barrel_cooling_value = 0;
    robot_status.shooter_barrel_heat_limit = static_cast<uint16_t>(max_heat_);
    robot_status.shooter_17mm_1_barrel_heat = 0;
    robot_status.robot_pos.orientation.w = 1.0;
    robot_status.projectile_allowance_17mm = static_cast<uint16_t>(max_ammo_);
    robot_status_pub_->publish(robot_status);

    pb_rm_interfaces::msg::GameRobotHP all_hp;
    assignSentryHp(all_hp, 1, robot_status.current_hp);
    hp_pub_->publish(all_hp);

    rfid_pub_->publish(pb_rm_interfaces::msg::RfidStatus());
    event_pub_->publish(pb_rm_interfaces::msg::EventData());
    ground_robot_pub_->publish(pb_rm_interfaces::msg::GroundRobotPosition());
    buff_pub_->publish(pb_rm_interfaces::msg::Buff());
  }

  static uint8_t normalizeGameProgress(uint8_t raw_progress) {
    if (raw_progress <= pb_rm_interfaces::msg::GameStatus::GAME_OVER) {
      return raw_progress;
    }
    return pb_rm_interfaces::msg::GameStatus::RUNNING;
  }

  static void assignSentryHp(
    pb_rm_interfaces::msg::GameRobotHP &all_hp, uint8_t mode, uint16_t current_hp) {
    if (mode == 1) {
      all_hp.red_7_robot_hp = current_hp;
    } else {
      all_hp.blue_7_robot_hp = current_hp;
    }
  }

  int robot_id_;
  int max_hp_;
  int max_heat_;
  int max_ammo_;
  int default_game_progress_;
  bool publish_empty_topics_;
  int last_hp_{-1};
  bool seen_serial_{false};
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr hp_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::EventData>::SharedPtr event_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GroundRobotPosition>::SharedPtr ground_robot_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::Buff>::SharedPtr buff_pub_;
  rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fyt::serial_driver

RCLCPP_COMPONENTS_REGISTER_NODE(fyt::serial_driver::SentryRefereeBridgeNode)
