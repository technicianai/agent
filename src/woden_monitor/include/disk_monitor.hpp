#ifndef DISK_MONITOR_H
#define DISK_MONITOR_H

#include "mqtt_facade.hpp"

#include "rclcpp/rclcpp.hpp"

#include <nlohmann/json.hpp>

#include <functional>
#include <string>
#include <vector>

using namespace std;

namespace woeden
{
struct mount
{
  string path;
  long available;
  long total;

  nlohmann::json to_json();
};

class disk_monitor : public rclcpp::Node
{
public:
  disk_monitor(mqtt_facade facade);

  vector<mount> get_mounts();
  long remaining(string path);

private:
  void sample();

  vector<mount> mounts_;
  mqtt_facade facade_;
  rclcpp::TimerBase::SharedPtr timer_;

  const uint8_t SAMPLING_INTERVAL = 5;
  const double MINIMUM_DRIVE_SPACE_PERCENTAGE = 0.03;
};
}

#endif
