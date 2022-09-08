#ifndef ROS2_MONITOR_H
#define ROS2_MONITOR_H

#include "mqtt_facade.hpp"

#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

using namespace std;

namespace woeden
{
struct package
{
  string name;
  vector<string> executables;
};

struct robot_config
{
  vector<package> packages;
};

struct node
{
  string name;
  string status;
};

struct topic
{
  string name;
  string type;
  double frequency;
  uint32_t message_count;
  //vector<recording_trigger> triggers;
};

class ros2_monitor : public rclcpp::Node
{
public:
  ros2_monitor(mqtt_facade facade);

private:
  void discover_packages();
  void discover_nodes();
  void discover_topics();
  void sample_topic_freqs();

  mqtt_facade facade_;

  vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  vector<node> nodes_;
  vector<topic*> topics_;
  //vector<recording_trigger> triggers_;

  rclcpp::TimerBase::SharedPtr alive_timer_;
  rclcpp::TimerBase::SharedPtr nodes_timer_;
  rclcpp::TimerBase::SharedPtr topics_timer_;
  rclcpp::TimerBase::SharedPtr topics_freq_timer_;

  const uint8_t SAMPLING_INTERVAL = 3;
};
}

#endif
