#ifndef ROS2_MONITOR_H
#define ROS2_MONITOR_H

#include "mqtt_facade.hpp"
#include "recording_manager.hpp"
#include "ros2_dto.hpp"

#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "std_msgs/msg/string.hpp"

namespace woeden
{
class ros2_monitor : public rclcpp::Node
{
public:
  ros2_monitor(std::shared_ptr<mqtt_facade> facade, std::shared_ptr<recording_manager> rm, std::vector<recording_trigger> triggers);

  void add_trigger(recording_trigger rt);
  void update_trigger(uint32_t id, bool enabled);
  void open_gateway(std::string ec2_ip);
  void close_gateway();

private:
  void discover_packages();
  void discover_nodes();
  void discover_topics();
  void sample_topic_freqs();
  void default_callback(std::shared_ptr<rclcpp::SerializedMessage> msg, topic* t);
  void json_trigger_callback(std::shared_ptr<std_msgs::msg::String> msg, topic* t);
  void key_value_trigger_callback(std::shared_ptr<diagnostic_msgs::msg::KeyValue> msg, topic* t);
  void status_trigger_callback(std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> msg, topic* t);
  void status_array_trigger_callback(std::shared_ptr<diagnostic_msgs::msg::DiagnosticArray> msg, topic* t);
  void gateway_cmd(std::string ec2_ip);
  void rosbridge_server_cmd();

  std::shared_ptr<mqtt_facade> facade_;
  std::shared_ptr<recording_manager> rm_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  std::vector<node> nodes_;
  std::vector<topic*> topics_;
  std::vector<recording_trigger> unassigned_triggers_;

  rclcpp::TimerBase::SharedPtr alive_timer_;
  rclcpp::TimerBase::SharedPtr nodes_timer_;
  rclcpp::TimerBase::SharedPtr topics_timer_;
  rclcpp::TimerBase::SharedPtr topics_freq_timer_;

  pid_t gateway_pid_;
  pid_t rosbridge_server_pid_;
  bool gateway_open_;

  const uint8_t SAMPLING_INTERVAL = 3;
};
}

#endif
