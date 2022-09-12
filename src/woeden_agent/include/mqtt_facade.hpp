#ifndef MQTT_FACADE_H
#define MQTT_FACADE_H

#include "disk_dto.hpp"
#include "recording_dto.hpp"
#include "recording_trigger.hpp"
#include "ros2_dto.hpp"
#include "mqtt/async_client.h"

#include <nlohmann/json.hpp>

#include <functional>
#include <string>
#include <vector>

namespace woeden
{
class mqtt_facade
{
public:
  mqtt_facade(std::string host, uint64_t robot_id, std::string password);
  ~mqtt_facade();

  void publish_alive();
  void publish_robot_config(robot_config rc);
  void publish_nodes(std::vector<node> nodes);
  void publish_topics(std::vector<topic> topics);

  void publish_mounted_paths(std::vector<mount> mounts);

  void publish_started(uint64_t bag_id);
  void publish_stopped(uint64_t bag_id, recording_metadata rm);
  void publish_status(uint64_t bag_id, recording_status rs);

  void publish_uploaded(std::string data);

  void publish_autostart(uint64_t recording_trigger_id);

  void set_record_callback(std::function<void (uint32_t, std::string, uint32_t, std::vector<recording_topic>)> cb);
  void set_stop_callback(std::function<void ()> cb);
  void set_upload_callback(std::function<void (uint32_t, std::string, std::vector<std::string>)> cb);
  void set_new_trigger_callback(std::function<void (recording_trigger)> cb);

private:
  void dispatch(mqtt::const_message_ptr msg);
  void publish(std::string topic, nlohmann::json payload);
  void publish(std::string topic, const char* payload);
  std::string mqtt_topic(std::string suffix);

  mqtt::async_client_ptr client_;
  std::string robot_id_str_;
  std::string password_;

  std::function<void (uint32_t, std::string, uint32_t, std::vector<recording_topic>)> on_record_;
  std::function<void ()> on_stop_;
  std::function<void (uint32_t, std::string, std::vector<std::string>)> on_upload_;
  std::function<void (recording_trigger)> on_new_trigger_;
};
}

#endif
