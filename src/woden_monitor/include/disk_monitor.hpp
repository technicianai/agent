#ifndef DISK_MONITOR_H
#define DISK_MONITOR_H

#include "mqtt_facade.hpp"
#include "disk_dto.hpp"

#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <string>
#include <vector>

namespace woeden
{
class disk_monitor : public rclcpp::Node
{
public:
  disk_monitor(std::shared_ptr<mqtt_facade> facade);

  std::vector<mount> get_mounts();
  long remaining(std::string path);

private:
  void sample();

  std::vector<mount> mounts_;
  std::shared_ptr<mqtt_facade> facade_;
  rclcpp::TimerBase::SharedPtr timer_;

  const uint8_t SAMPLING_INTERVAL = 5;
  const double MINIMUM_DRIVE_SPACE_PERCENTAGE = 0.03;
};
}

#endif
