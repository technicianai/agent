#ifndef ROS2_MONITOR_H
#define ROS2_MONITOR_H

#include "mqtt_facade.hpp"
#include "ros2_dto.hpp"

#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "std_msgs/msg/string.hpp"

namespace woeden
{
class ros2_monitor : public rclcpp::Node
{
public:
  ros2_monitor(std::shared_ptr<mqtt_facade> facade, std::vector<recording_trigger> triggers);

  void add_trigger(recording_trigger rt);
  void open_gateway(std::string ec2_ip);
  void close_gateway();

private:
  void discover_packages();
  void discover_nodes();
  void discover_topics();
  void sample_topic_freqs();
  void default_callback(std::shared_ptr<rclcpp::SerializedMessage> msg, topic* t);
  void trigger_callback(std::shared_ptr<std_msgs::msg::String> msg, topic* t);
  void gateway_cmd(std::string ec2_ip);
  void rosbridge_server_cmd();

  std::shared_ptr<mqtt_facade> facade_;

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
