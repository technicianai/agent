#ifndef RECORDING_MANAGER_H
#define RECORDING_MANAGER_H

#include "disk_monitor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <string>
#include <vector>

#include <sys/types.h>
#include <unistd.h>

using namespace std;

namespace woeden
{
struct recording_topic
{
  string name;
  bool throttle;
  double frequency;
};

struct recording_status
{
  double eta;
  double rate;
  uintmax_t size;
};

struct recording_metadata
{
  string metadata;
  uintmax_t size;
};

class recording_manager : public rclcpp::Node
{
public:
  recording_manager(shared_ptr<disk_monitor> dm, mqtt_facade facade);

  void start(uint32_t bag_id, string base_path, vector<recording_topic> recording_topics);
  void stop();
  bool is_recording();
  uintmax_t bag_size();

private:
  void throttle_cmd(string topic, double frequency);
  void record_cmd(string bag_path, vector<recording_topic> recording_topics);

  void remote_throttle_from_db(const char* db3_path);
  string load_metadata(string metadata_path);
  string remote_throttle_from_metadata(string metadata);
  void update_metadata(string metadata_path, string metadata);

  uintmax_t directory_size(string path);

  void status_check();

  bool recording_;

  pid_t recording_pid_;
  pid_t upload_pid_;
  vector<pid_t> throttle_pids_;

  string base_path_;
  string bag_path_;

  uint32_t bag_id_;

  uintmax_t size_;
  uintmax_t previous_size_;

  rclcpp::TimerBase::SharedPtr timer_;

  shared_ptr<disk_monitor> dm_;
  mqtt_facade facade_;

  const int SAMPLING_INTERVAL = 1;
};
}

#endif
