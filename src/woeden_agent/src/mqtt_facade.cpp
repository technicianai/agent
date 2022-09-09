#include "mqtt_facade.hpp"
#include "ros2_monitor.hpp"
#include "utils.hpp"

#include <nlohmann/json.hpp>

#include <vector>

using namespace std;

namespace woeden
{
mqtt_facade::mqtt_facade(string host, uint64_t robot_id, string password)
{
  robot_id_str_ = std::to_string(robot_id);
  password_ = password;
  client_ = make_shared<mqtt::async_client>(host, robot_id_str_);

  auto connect_options = mqtt::connect_options_builder() 
    .user_name(robot_id_str_)
    .password(password_)
    .keep_alive_interval(std::chrono::seconds(20))
    .automatic_reconnect(std::chrono::seconds(2), std::chrono::seconds(30))
    .clean_session()
    .finalize();

  client_->set_message_callback(bind(&mqtt_facade::dispatch, this, placeholders::_1));
  mqtt::token_ptr token_conn = client_->connect(connect_options);
  token_conn->wait();

  mqtt::subscribe_options subOpts;
  mqtt::properties props {
    { mqtt::property::SUBSCRIPTION_IDENTIFIER, 1 },
  };
  try {
    client_->subscribe(mqtt_topic("record"), 0, subOpts, props);
    client_->subscribe(mqtt_topic("stop"), 0, subOpts, props);
    client_->subscribe(mqtt_topic("upload"), 0, subOpts, props);
    //client_->subscribe(new_trigger_topic, 0, subOpts, props);
  } catch (mqtt::exception& e) { 
    cout << e.what() << endl; 
  }

  client_->start_consuming();

  cout << "Successfully connected to Woeden. Actively monitoring." << endl;
}

mqtt_facade::~mqtt_facade()
{
  client_->disconnect();
  client_.reset();
}

void mqtt_facade::publish_alive()
{
  publish("alive", "1");
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
  publish("config", data);
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
  publish("nodes", nodes_json);
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
  publish("topics", topics_json);
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
  publish("mounted_paths", mounts_json);
}

void mqtt_facade::publish_started(uint64_t bag_id)
{
  nlohmann::json data;
  data["id"] = bag_id;
  publish("started", data);
}

void mqtt_facade::publish_stopped(uint64_t bag_id, recording_metadata rm)
{
  nlohmann::json data;
  data["id"] = bag_id;
  data["yaml"] = rm.metadata;
  data["size"] = rm.size;
  publish("stopped", data);
}

void mqtt_facade::publish_status(uint64_t bag_id, recording_status rs)
{
  nlohmann::json data;
  data["id"] = bag_id;
  data["size"] = rs.size;
  data["rate"] = rs.rate;
  data["eta"] = rs.eta;
  publish("recording", data);
}

void mqtt_facade::publish_uploaded(string data)
{
  publish("uploaded", data.c_str());
}

void mqtt_facade::publish(string topic, nlohmann::json payload)
{
  publish(topic, payload.dump().c_str());
}

void mqtt_facade::publish(string topic, const char* payload)
{
  try {
  mqtt::message_ptr msg = mqtt::make_message(mqtt_topic(topic), payload);
  client_->publish(msg);
  } catch (mqtt::exception& e) { 
    cout << e.what() << endl; 
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

    uint32_t bag_id = data["id"].get<uint32_t>();
    string base_path = data["base_path"];
    std::vector<recording_topic> recording_topics;

    for (auto& topic : data["topics"]) {
      recording_topics.push_back({
        .name = topic["name"].get<std::string>(),
        .throttle = topic["max_frequency"].get<bool>(),
        .frequency = topic["frequency"].get<double>()
      });
    }

    on_record_(bag_id, base_path, recording_topics);
  } else if (topic == mqtt_topic("stop")) {
    on_stop_();
  } else if (topic == mqtt_topic("upload")) {
    nlohmann::json data = nlohmann::json::parse(payload);

    uint32_t bag_id = data["id"].get<uint32_t>();
    std::string base_path = data["base_path"];
    std::vector<std::string> urls;
    
    for (auto& url : data["urls"]) {
      urls.push_back(url.get<std::string>());
    }

    on_upload_(bag_id, base_path, urls);
  }
  // else if (topic == new_trigger_topic) {
  //   std::string payload = msg->to_string();
  //   nlohmann::json data = nlohmann::json::parse(payload);
  //   recording_trigger rt = recording_trigger::from_json(data);
  //   c_.add_recording_trigger(rt);
  //   triggers_.push_back(rt);
  // }
}

void mqtt_facade::set_record_callback(function<void (uint32_t, string, vector<recording_topic>)> cb)
{
  on_record_ = cb;
}

void mqtt_facade::set_stop_callback(function<void ()> cb)
{
  on_stop_ = cb;
}

void mqtt_facade::set_upload_callback(function<void (uint32_t, string, vector<string>)> cb)
{
  on_upload_ = cb;
}
}