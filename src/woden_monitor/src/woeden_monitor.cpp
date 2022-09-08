#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <thread>
#include <filesystem>

#include <sqlite3.h> 
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_event.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mqtt/async_client.h"
#include "mqtt/connect_options.h"

#include "disk_monitor.hpp"
#include "recording_manager.hpp"
#include "recording_trigger.hpp"
#include "utils.hpp"
#include "config.hpp"
#include "mqtt_facade.hpp"
#include "ros2_monitor.hpp"

using namespace woeden;

class woeden_monitor : public rclcpp::Node
{
public:
  woeden_monitor(std::string host, uint32_t robot_id, std::shared_ptr<recording_manager> rm, mqtt::async_client_ptr client) : Node("woeden_monitor"), client_(client)
  {
    // triggers_ = c_.get_recording_triggers();
    rm_ = rm;
    dm_ = dm;
    robot_id_str_ = std::to_string(robot_id);
    sleep(5);
    std::function<void ()> consume_callback = std::bind(
      &woeden_monitor::consume_message, 
      this,
      mqtt_topic("record"),
      mqtt_topic("stop"),
      mqtt_topic("upload")
    );
    consume_timer_ = this->create_wall_timer(std::chrono::seconds(5), consume_callback);
    std::cout << "Connection to Woeden was successful. Actively monitoring." << std::endl;
  }

private:
  void mqtt_topic(string suffix)
  {
    return "/" + robot_id_str_ + "/" + suffix;
  }

  void consume_message(std::string start_topic, std::string stop_topic, std::string upload_topic)
  {
    mqtt::const_message_ptr msg;
    if (!client_->try_consume_message(&msg)) return;

    if (msg->get_topic() == start_topic) {
      std::string payload = msg->to_string();
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

      rm_->start(bag_id, base_path, recording_topics);
    } else if (msg->get_topic() == stop_topic) {
      rm_->stop();
    } else if (msg->get_topic() == upload_topic) {
      std::string payload = msg->to_string();
      nlohmann::json data = nlohmann::json::parse(payload);
      uint32_t bag_id = data["id"].get<uint32_t>();
      std::string base_path = data["base_path"];
      std::vector<std::string> urls;
      for (auto& url : data["urls"]) {
        urls.push_back(url.get<std::string>());
      }
      upload_bag(bag_id, base_path, urls, [&, uploaded_topic](std::string output) -> void {
        mqtt::message_ptr msg = mqtt::make_message(uploaded_topic, output.c_str());
        client_->publish(msg);
      });
    } 
    // else if (msg->get_topic() == new_trigger_topic) {
    //   std::string payload = msg->to_string();
    //   nlohmann::json data = nlohmann::json::parse(payload);
    //   recording_trigger rt = recording_trigger::from_json(data);
    //   c_.add_recording_trigger(rt);
    //   triggers_.push_back(rt);
    // }
  }

  rclcpp::TimerBase::SharedPtr consume_timer_;
  mqtt::async_client_ptr client_;
  std::shared_ptr<recording_manager> rm_;
  string robot_id_str_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string host = getenv("HOST");
  std::string home = getenv("HOME");

  config c(home);

  mqtt_facade facade(host, c.get_id(), c.get_password());

  auto dm = std::make_shared<disk_monitor>(facade);
  auto rm = std::make_shared<recording_manager>(dm, facade);
  auto r2m = std::make_shared<ros2_monitor>(facade);
  auto wm = std::make_shared<woeden_monitor>(host, c.get_id(), rm, facade.client_);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dm);
  executor.add_node(rm);
  executor.add_node(r2m);
  executor.add_node(wm);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
