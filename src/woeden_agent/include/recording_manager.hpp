#ifndef RECORDING_MANAGER_H
#define RECORDING_MANAGER_H

#include "disk_monitor.hpp"
#include "mqtt_facade.hpp"
#include "recording_dto.hpp"
#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <string>
#include <vector>

#include <sys/types.h>
#include <unistd.h>

namespace woeden
{
class recording_manager : public rclcpp::Node
{
public:
  recording_manager(std::shared_ptr<disk_monitor> dm, std::shared_ptr<mqtt_facade> facade);

  void start(uint32_t bag_id, std::string base_path, uint32_t duration, std::vector<recording_topic> recording_topics);
  void stop();
  bool is_recording();
  uintmax_t bag_size();

  void upload(uint32_t bag_id, std::string base_path, std::vector<std::string> urls);

private:
  void throttle_cmd(std::string topic, double frequency);
  void record_cmd(std::string bag_path, std::vector<recording_topic> recording_topics);

  void remote_throttle_from_db(const char* db3_path);
  std::string load_metadata(std::string metadata_path);
  std::string remote_throttle_from_metadata(std::string metadata);
  void update_metadata(std::string metadata_path, std::string metadata);

  uintmax_t directory_size(std::string path);

  void status_check();

  bool recording_;

  pid_t recording_pid_;
  pid_t upload_pid_;
  std::vector<pid_t> throttle_pids_;

  std::string base_path_;
  std::string bag_path_;

  uint32_t bag_id_;

  uintmax_t size_;
  uintmax_t previous_size_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr auto_stop_timer_;

  std::shared_ptr<disk_monitor> dm_;
  std::shared_ptr<mqtt_facade> facade_;

  const int SAMPLING_INTERVAL = 1;
};
}

#endif
