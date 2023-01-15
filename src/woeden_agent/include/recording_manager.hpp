#ifndef RECORDING_MANAGER_H
#define RECORDING_MANAGER_H

#include "config.hpp"
#include "disk_monitor.hpp"
#include "mqtt_facade.hpp"
#include "recording_dto.hpp"
#include "recording_trigger.hpp"
#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/upload_bytes.hpp"
#include "interfaces/srv/upload.hpp"
#include "interfaces/srv/upload_complete.hpp"

#include <functional>
#include <string>
#include <vector>

#include <time.h>
#include <sys/types.h>
#include <unistd.h>

namespace woeden
{
class recording_manager : public rclcpp::Node
{
public:
  recording_manager(std::shared_ptr<disk_monitor> dm, std::shared_ptr<mqtt_facade> facade, always_record_config arc, double max_bandwidth);

  void start(std::string bag_uuid, std::string base_path, uint32_t duration, std::vector<recording_topic> recording_topics);
  void auto_start(recording_trigger rt);
  void stop();
  bool is_recording();
  uintmax_t bag_size();
  uintmax_t bag_size(std::string bag_path);

  void set_always_record(always_record_config arc);

  void upload(std::string bag_uuid, std::string base_path);
  void metadata_on_reconnect();

  void gif_upload(std::string bag_uuid, std::string base_path, std::string urls);

  void set_max_bandwidth(double bw);

private:
  void throttle_cmd(std::string topic, double frequency);
  void always_record_cmd(std::string bag_path);
  void record_cmd(std::string bag_path, std::vector<recording_topic> recording_topics);
  void start_always_record();
  void stop_always_record();
  void always_record();
  void annihilate_recording(pid_t pid, std::string bag_path);
  void upload_bytes(std::shared_ptr<interfaces::msg::UploadBytes> msg);
  void upload_complete(std::shared_ptr<interfaces::srv::UploadComplete::Request> request, std::shared_ptr<interfaces::srv::UploadComplete::Response> response);

  void remote_throttle_from_db(const char* db3_path);
  std::string load_metadata(std::string metadata_path);
  std::string remote_throttle_from_metadata(std::string metadata);
  void update_metadata(std::string metadata_path, std::string metadata);

  uintmax_t directory_size(std::string path);

  void status_check();

  bool recording_;
  bool stopping_;

  pid_t recording_pid_;
  pid_t upload_pid_;
  std::vector<pid_t> throttle_pids_;

  pid_t always_record_pid_1_;
  pid_t always_record_pid_2_;

  bool always_record_turn_;

  std::string always_record_bag_path_1_;
  std::string always_record_bag_path_2_;

  std::string always_record_bag_uuid_1_;
  std::string always_record_bag_uuid_2_;

  always_record_config always_record_config_;

  std::string base_path_;
  std::string bag_path_;

  std::string bag_uuid_;
  uint32_t trigger_id_;

  time_t trigger_start_time_;
  time_t trigger_duration_;

  uintmax_t size_;
  uintmax_t previous_size_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr auto_stop_timer_;
  rclcpp::TimerBase::SharedPtr always_record_timer_;

  rclcpp::Client<interfaces::srv::Upload>::SharedPtr upload_client_;
  rclcpp::Subscription<interfaces::msg::UploadBytes>::SharedPtr upload_subscription_;
  rclcpp::Service<interfaces::srv::UploadComplete>::SharedPtr upload_complete_;

  std::shared_ptr<disk_monitor> dm_;
  std::shared_ptr<mqtt_facade> facade_;

  double max_bandwidth_;

  const int SAMPLING_INTERVAL = 1;
};
}

#endif
