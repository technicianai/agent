#ifndef ROS2_MONITOR_H
#define ROS2_MONITOR_H

#include "mqtt_facade.hpp"
#include "ros2_dto.hpp"

#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace woeden
{
class ros2_monitor : public rclcpp::Node
{
public:
  ros2_monitor(std::shared_ptr<mqtt_facade> facade);

private:
  void discover_packages();
  void discover_nodes();
  void discover_topics();
  void sample_topic_freqs();

  std::shared_ptr<mqtt_facade> facade_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  std::vector<node> nodes_;
  std::vector<topic*> topics_;
  //vector<recording_trigger> triggers_;

  rclcpp::TimerBase::SharedPtr alive_timer_;
  rclcpp::TimerBase::SharedPtr nodes_timer_;
  rclcpp::TimerBase::SharedPtr topics_timer_;
  rclcpp::TimerBase::SharedPtr topics_freq_timer_;

  const uint8_t SAMPLING_INTERVAL = 3;
};
}

#endif
