#include "config.hpp"
#include "mqtt_facade.hpp"
#include "recording_dto.hpp"
#include "ros2_monitor.hpp"
#include "utils.hpp"

#include <nlohmann/json.hpp>

#include <vector>

using namespace std;

namespace woeden
{
mqtt_facade::mqtt_facade(string host, uint64_t robot_id, string password)
{
  robot_id_str_ = to_string(robot_id);
  password_ = password;
  client_ = make_shared<mqtt::async_client>(host, robot_id_str_);
  is_connected_ = false;

  mqtt::ssl_options ssl_opts;
  ssl_opts.set_trust_store("/woeden_agent/certs/isrgrootx1.pem");

  connect_options_ = mqtt::connect_options_builder() 
    .user_name(robot_id_str_)
    .password(password_)
    .keep_alive_interval(chrono::seconds(20))
    .automatic_reconnect(chrono::seconds(2), chrono::seconds(30))
    .clean_session()
    .ssl(move(ssl_opts))
    .finalize();

  client_->set_message_callback(bind(&mqtt_facade::dispatch, this, placeholders::_1));

  client_->set_connected_handler([&](string data) -> void {
    mqtt::subscribe_options subOpts;
    mqtt::properties props { { mqtt::property::SUBSCRIPTION_IDENTIFIER, 1 } };
    client_->subscribe(mqtt_topic("record"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("stop"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("upload"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("new_trigger"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("robot_gateway"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("robot_gateway/close"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("update_trigger"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("update_always_record"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("gif/upload"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("update_max_bandwidth"), 2, subOpts, props);
    client_->subscribe(mqtt_topic("metadata/upload"), 2, subOpts, props);
    on_reconnect_();
  });
}

mqtt_facade::~mqtt_facade()
{
  client_->disconnect();
  client_.reset();
}

void mqtt_facade::connect()
{
  bool failed_once = false;
  while (!is_connected_) {
    try {
      mqtt::token_ptr token_conn = client_->connect(connect_options_);
      token_conn->wait();
      client_->start_consuming();
      is_connected_ = true;
    } catch (mqtt::exception& e) {
      if (!failed_once) {
        cout << "Failed to establish a connection to Woeden. Actively retrying. Reason: ";
        cout << e.what();
        cout << endl;
        failed_once = true;
      }
    }
  }
}

void mqtt_facade::publish_alive()
{
  publish("alive", "1", 0);
}

void mqtt_facade::publish_robot_config(robot_config rc)
{
  nlohmann::json pkgs_json;
  for (package& pkg : rc.packages) {
    nlohmann::json pkg_json;
    pkg_json["package"] = pkg.name;
    pkg_json["executables"] = nlohmann::json(pkg.executables);
    pkgs_json.push_back(pkg_json);
  }
  nlohmann::json data;
  data["executables"] = pkgs_json;
  publish("config", data, 0);
}

void mqtt_facade::publish_nodes(vector<node> nodes)
{
  nlohmann::json nodes_json;
  for (node& n : nodes) {
    nlohmann::json node_json;
    node_json["name"] = n.name;
    node_json["status"] = n.status;
    nodes_json.push_back(node_json);
  }
  publish("nodes", nodes_json, 0);
}

void mqtt_facade::publish_topics(vector<topic> topics)
{
  nlohmann::json topics_json;
  for (topic& t : topics) {
    nlohmann::json topic_json;
    topic_json["name"] = t.name;
    topic_json["type"] = t.type;
    topic_json["frequency"] = t.frequency;
    topics_json.push_back(topic_json);
  }
  publish("topics", topics_json, 0);
}

void mqtt_facade::publish_mounted_paths(vector<mount> mounts)
{
  nlohmann::json mounts_json;
  for (mount & mnt : mounts) {
    nlohmann::json mount_json;
    mount_json["path"] = mnt.path;
    mount_json["available"] = mnt.available;
    mount_json["total"] = mnt.total;
    mounts_json.push_back(mount_json);
  }
  publish("mounted_paths", mounts_json, 0);
}

void mqtt_facade::publish_started(string bag_uuid, uint32_t trigger_id)
{
  nlohmann::json data;
  data["bag_uuid"] = bag_uuid;
  data["trigger_id"] = trigger_id;
  publish("started", data, 2);
}

void mqtt_facade::publish_stopped(string bag_uuid, uintmax_t size)
{
  nlohmann::json data;
  data["bag_uuid"] = bag_uuid;
  data["size"] = size;
  publish("stopped", data, 2);
}

void mqtt_facade::publish_metadata(string bag_uuid, string metadata)
{
  nlohmann::json data;
  data["bag_uuid"] = bag_uuid;
  data["metadata"] = metadata;
  publish("metadata", data, 2);
}

void mqtt_facade::publish_status(string bag_uuid, recording_status rs)
{
  nlohmann::json data;
  data["bag_uuid"] = bag_uuid;
  data["size"] = rs.size;
  data["rate"] = rs.rate;
  data["eta"] = rs.eta;
  data["trigger_id"] = rs.trigger_id;
  publish("recording", data, 1);
}

void mqtt_facade::publish_uploaded(string data)
{
  publish("uploaded", data.c_str(), 2);
}

void mqtt_facade::publish_gateway_open()
{
  publish("robot_gateway/opened", "", 2);
}

void mqtt_facade::publish_gateway_closed()
{
  publish("robot_gateway/closed", "", 2);
}

void mqtt_facade::publish_trigger_status(vector<recording_trigger> triggers)
{
  nlohmann::json data = nlohmann::json::array();
  for (recording_trigger rt : triggers) {
    data.push_back(rt.to_reduced_json());
  }
  publish("triggers", data, 1);
}

void mqtt_facade::publish_always_record_status(always_record_config arc)
{
  nlohmann::json data;
  data["enabled"] = arc.enabled;
  data["base_path"] = arc.base_path;
  data["duration"] = arc.duration;
  publish("always_record", data, 2);
}

void mqtt_facade::publish_max_bandwidth_status(double max_bandwidth)
{
  publish("max_bandwidth", to_string(max_bandwidth).c_str(), 2);
}

void mqtt_facade::publish_gif_uploaded(string bag_uuid)
{
  nlohmann::json data;
  data["bag_uuid"] = bag_uuid;
  publish("gif/uploaded", data, 2);
}

void mqtt_facade::publish_chunk(string bag_uuid, uint32_t index, const void * contents, size_t len)
{
  string topic = "bag/" + bag_uuid + "/upload/" + to_string(index);
  publish(topic, contents, len, 1);
}

void mqtt_facade::publish_upload_complete(string bag_uuid, uint32_t num_chunks)
{
  nlohmann::json data;
  data["bag_uuid"] = bag_uuid;
  data["num_chunks"] = num_chunks;
  string topic = "bag/" + bag_uuid + "/upload/complete";
  publish(topic, data, 2);
}

void mqtt_facade::publish(string topic, nlohmann::json payload, int qos)
{
  publish(topic, payload.dump().c_str(), qos);
}

void mqtt_facade::publish(string topic, const char* payload, int qos)
{
  try {
    mqtt::message_ptr msg = mqtt::make_message(mqtt_topic(topic), payload);
    msg->set_qos(qos);
    client_->publish(msg);
    if (!is_connected_) {
      cout << "Successfully connected to Woeden. Actively monitoring." << endl; 
      is_connected_ = true;
    }
  } catch (mqtt::exception& e) {
    if (is_connected_) {
      cout << "Disconnected from Woeden. Attempting to reconnect. Reason: ";
      cout << e.what();
      cout << endl;
      is_connected_ = false;
    }
  }
}

void mqtt_facade::publish(string topic, const void* payload, size_t len, int qos)
{
  try {
    mqtt::message_ptr msg = mqtt::make_message(mqtt_topic(topic), payload, len);
    msg->set_qos(qos);
    client_->publish(msg);
    if (!is_connected_) {
      cout << "Successfully connected to Woeden. Actively monitoring." << endl; 
      is_connected_ = true;
    }
  } catch (mqtt::exception& e) {
    if (is_connected_) {
      cout << "Disconnected from Woeden. Attempting to reconnect. Reason: ";
      cout << e.what();
      cout << endl;
      is_connected_ = false;
    }
  }
}

string mqtt_facade::mqtt_topic(string suffix)
{
  return "/" + robot_id_str_ + "/" + suffix;
}

void mqtt_facade::dispatch(mqtt::const_message_ptr msg)
{
  string topic = msg->get_topic();
  string payload = msg->to_string();

  if (topic == mqtt_topic("record")) {
    nlohmann::json data = nlohmann::json::parse(payload);

    string bag_uuid = data["bag_uuid"];
    string base_path = data["base_path"];
    uint32_t duration = data["duration"].get<uint32_t>();
    vector<recording_topic> recording_topics;
    for (auto& topic : data["topics"]) {
      recording_topics.push_back(recording_topic::from_json(topic));
    }

    on_record_(bag_uuid, base_path, duration, recording_topics);
  } else if (topic == mqtt_topic("stop")) {
    on_stop_(false);
  } else if (topic == mqtt_topic("upload")) {
    nlohmann::json data = nlohmann::json::parse(payload);
    string bag_uuid = data["bag_uuid"];
    string base_path = data["base_path"];
    on_upload_(bag_uuid, base_path);
  } else if (topic == mqtt_topic("new_trigger")) {
    nlohmann::json data = nlohmann::json::parse(payload);
    recording_trigger rt = recording_trigger::from_json(data);
    on_new_trigger_(rt);
  } else if (topic == mqtt_topic("robot_gateway")) {
    nlohmann::json data = nlohmann::json::parse(payload);
    string ec2_ip = data["ec2_ip"].get<string>();
    on_gateway_(ec2_ip);
  } else if (topic == mqtt_topic("robot_gateway/close")) {
    on_gateway_close_();
  } else if (topic == mqtt_topic("update_trigger")) {
    nlohmann::json data = nlohmann::json::parse(payload);
    uint32_t id = data["id"].get<uint32_t>();
    bool enabled = data["enabled"].get<bool>();
    on_update_trigger_(id, enabled);
  } else if (topic == mqtt_topic("update_always_record")) {
    nlohmann::json data = nlohmann::json::parse(payload);
    bool enabled = data["enabled"].get<bool>();
    uint32_t duration = data["duration"].get<uint32_t>();
    string base_path = data["base_path"];
    on_update_always_record_(duration, enabled, base_path);
  } else if (topic == mqtt_topic("gif/upload")) {
    nlohmann::json data = nlohmann::json::parse(payload);
    string bag_uuid = data["bag_uuid"];
    string base_path = data["base_path"];
    string urls = data["urls"].dump();
    on_gif_upload_(bag_uuid, base_path, urls);
  } else if (topic == mqtt_topic("update_max_bandwidth")) {
    on_update_max_bandwidth_(stod(payload));
  } else if (topic == mqtt_topic("metadata/upload")) {
    nlohmann::json data = nlohmann::json::parse(payload);
    string bag_uuid = data["bag_uuid"];
    string base_path = data["base_path"];
    on_metadata_upload_(bag_uuid, base_path);
  }
}

void mqtt_facade::set_record_callback(function<void (string, string, uint32_t, vector<recording_topic>)> cb)
{
  on_record_ = cb;
}

void mqtt_facade::set_stop_callback(function<void (bool)> cb)
{
  on_stop_ = cb;
}

void mqtt_facade::set_upload_callback(function<void (string, string)> cb)
{
  on_upload_ = cb;
}

void mqtt_facade::set_new_trigger_callback(function<void (recording_trigger)> cb)
{
  on_new_trigger_ = cb;
}

void mqtt_facade::set_update_trigger_callback(function<void (uint32_t, bool)> cb)
{
  on_update_trigger_ = cb;
}

void mqtt_facade::set_update_always_record_callback(function<void (uint32_t, bool, string)> cb)
{
  on_update_always_record_ = cb;
}

void mqtt_facade::set_gateway_callback(function<void (string)> cb)
{
  on_gateway_ = cb;
}

void mqtt_facade::set_gateway_close_callback(function<void ()> cb)
{
  on_gateway_close_ = cb;
}

void mqtt_facade::set_reconnect_callback(function<void ()> cb)
{
  on_reconnect_ = cb;
}

void mqtt_facade::set_gif_upload_callback(function<void (string, string, string)> cb)
{
  on_gif_upload_ = cb;
}

void mqtt_facade::set_update_max_bandwidth_callback(function<void (double)> cb)
{
  on_update_max_bandwidth_ = cb;
}

void mqtt_facade::set_metadata_upload_callback(function<void (string, string)> cb)
{
  on_metadata_upload_ = cb;
}
}
