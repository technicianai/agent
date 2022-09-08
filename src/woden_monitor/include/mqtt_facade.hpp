#ifndef MQTT_FACADE_H
#define MQTT_FACADE_H

#include "disk_dto.hpp"
#include "recording_dto.hpp"
#include "ros2_dto.hpp"
#include "mqtt/async_client.h"

#include <nlohmann/json.hpp>

#include <functional>
#include <string>
#include <vector>

using namespace std;

namespace woeden
{
class mqtt_facade
{
public:
  mqtt_facade(string host, uint64_t robot_id, string password);
  ~mqtt_facade();

  void publish_alive();
  void publish_robot_config(robot_config rc);
  void publish_nodes(vector<node> nodes);
  void publish_topics(vector<topic> topics);

  void publish_mounted_paths(vector<mount> mounts);

  void publish_started(uint64_t bag_id);
  void publish_stopped(uint64_t bag_id, recording_metadata rm);
  void publish_status(uint64_t bag_id, recording_status rs);

  void publish_uploaded(string data);

  void set_record_callback(function<void (uint32_t, string, vector<recording_topic>)> cb);
  void set_stop_callback(function<void ()> cb);
  void set_upload_callback(function<void (uint32_t, string, vector<string>)> cb);

private:
  void dispatch(mqtt::const_message_ptr msg);
  void publish(string topic, nlohmann::json payload);
  void publish(string topic, const char* payload);
  string mqtt_topic(string suffix);

  mqtt::async_client_ptr client_;
  string robot_id_str_;
  string password_;

  function<void (uint32_t, string, vector<recording_topic>)> on_record_;
  function<void ()> on_stop_;
  function<void (uint32_t, string, vector<string>)> on_upload_;
};
}

#endif
