#ifndef MQTT_FACADE_H
#define MQTT_FACADE_H

#include "config.hpp"
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

  void connect();
  void publish_alive();
  void publish_robot_config(robot_config rc);
  void publish_nodes(std::vector<node> nodes);
  void publish_topics(std::vector<topic> topics);

  void publish_mounted_paths(std::vector<mount> mounts);

  void publish_started(std::string bag_uuid, uint32_t trigger_id);
  void publish_stopped(std::string bag_uuid, uintmax_t size);
  void publish_status(std::string bag_uuid, recording_status rs);
  void publish_metadata(std::string bag_uuid, std::string metadata);

  void publish_uploaded(std::string data);
  void publish_upload_complete(std::string bag_uuid, uint32_t num_chunks);

  void publish_gateway_open();
  void publish_gateway_closed();

  void publish_trigger_status(std::vector<recording_trigger> triggers);
  void publish_always_record_status(always_record_config arc);
  void publish_max_bandwidth_status(double max_bandwidth);

  void publish_gif_uploaded(std::string bag_uuid);

  void publish_chunk(std::string bag_uuid, uint32_t index, const void* contents, size_t len);

  void set_record_callback(std::function<void (std::string, std::string, uint32_t, std::vector<recording_topic>)> cb);
  void set_stop_callback(std::function<void ()> cb);
  void set_upload_callback(std::function<void (std::string, std::string)> cb);
  void set_update_trigger_callback(std::function<void (uint32_t, bool)> cb);
  void set_update_always_record_callback(std::function<void (uint32_t, bool, std::string)> cb);
  void set_new_trigger_callback(std::function<void (recording_trigger)> cb);
  void set_gateway_callback(std::function<void (std::string)> cb);
  void set_gateway_close_callback(std::function<void ()> cb);
  void set_reconnect_callback(std::function<void ()> cb);
  void set_gif_upload_callback(std::function<void (std::string, std::string, std::string)> cb);
  void set_update_max_bandwidth_callback(std::function<void (double)> cb);
  void set_metadata_upload_callback(std::function<void (std::string, std::string)> cb);

private:
  void dispatch(mqtt::const_message_ptr msg);
  void publish(std::string topic, nlohmann::json payload, int qos);
  void publish(std::string topic, const char* payload, int qos);
  void publish(std::string topic, const void* payload, size_t len, int qos);
  std::string mqtt_topic(std::string suffix);

  mqtt::async_client_ptr client_;
  mqtt::connect_options connect_options_;
  bool is_connected_;
  std::string robot_id_str_;
  std::string password_;

  std::function<void (std::string, std::string, uint32_t, std::vector<recording_topic>)> on_record_;
  std::function<void ()> on_stop_;
  std::function<void (std::string, std::string)> on_upload_;
  std::function<void (recording_trigger)> on_new_trigger_;
  std::function<void (uint32_t, bool)> on_update_trigger_;
  std::function<void (uint32_t, bool, std::string)> on_update_always_record_;
  std::function<void (std::string)> on_gateway_;
  std::function<void ()> on_gateway_close_;
  std::function<void ()> on_reconnect_;
  std::function<void (std::string, std::string, std::string)> on_gif_upload_;
  std::function<void (double)> on_update_max_bandwidth_;
  std::function<void (std::string, std::string)> on_metadata_upload_;
};
}

#endif
