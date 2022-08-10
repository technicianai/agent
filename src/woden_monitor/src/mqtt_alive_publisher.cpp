#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <thread>
#include <filesystem>

#include <stdlib.h>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_event.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mqtt/async_client.h"
#include "mqtt/connect_options.h"

struct subscription
{
  std::string name;
  std::vector<std::string> types;
  int message_count;
};

class mqtt_alive_publisher : public rclcpp::Node
{
public:
  mqtt_alive_publisher() : Node("mqtt_alive_publisher"), client_("tcp://localhost:1883", "paho-cpp-data-publish", nullptr)
  {
    std::stringstream id_path;
    id_path << getenv("HOME");
    id_path << "/woden/id";
    std::ifstream id_file;
    id_file.open(id_path.str());
    int id;
    id_file >> id;

    std::stringstream bag_path;
    bag_path << getenv("HOME");
    bag_path << "/woden/bags/";
    
    std::string alive_topic = mqtt_topic(id, "alive");
    std::string executables_topic = mqtt_topic(id, "executables");
    std::string nodes_topic = mqtt_topic(id, "nodes");
    std::string topics_topic = mqtt_topic(id, "topics");
    std::string start_recording_topic = mqtt_topic(id, "record");
    std::string stop_recording_topic = mqtt_topic(id, "stop");
    std::string stopped_recording_topic = mqtt_topic(id, "stopped");
    std::string started_recording_topic = mqtt_topic(id, "started");
    std::string recording_topic = mqtt_topic(id, "recording");

    auto connect_options = mqtt::connect_options_builder() 
        .keep_alive_interval(std::chrono::seconds(20))
        .automatic_reconnect(std::chrono::seconds(2), std::chrono::seconds(30))
        .clean_session()
        .finalize();
    mqtt::subscribe_options subOpts;
    mqtt::properties props {
      { mqtt::property::SUBSCRIPTION_IDENTIFIER, 1 },
    };
    client_.connect(connect_options);
    try {
      client_.subscribe(start_recording_topic, 0, subOpts, props);
      client_.subscribe(stop_recording_topic, 0, subOpts, props);
    } catch (mqtt::exception e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }

    std::string executables = fetch_executables();
    mqtt::message_ptr msg = mqtt::make_message(executables_topic, executables.c_str());
    client_.publish(msg);
    
    std::function<void ()> alive_callback = std::bind(&mqtt_alive_publisher::timer_callback, this, alive_topic);
    timer_ = this->create_wall_timer(std::chrono::seconds(15), alive_callback);

    std::function<void ()> nodes_callback = std::bind(&mqtt_alive_publisher::update_nodes, this, nodes_topic);
    nodes_timer_ = this->create_wall_timer(std::chrono::seconds(15), nodes_callback);

    std::function<void ()> topic_discovery_callback = std::bind(&mqtt_alive_publisher::discover_topics, this);
    topic_discovery_timer_ = this->create_wall_timer(std::chrono::seconds(15), topic_discovery_callback);

    std::function<void ()> topics_callback = std::bind(&mqtt_alive_publisher::topics, this, topics_topic);
    topics_timer_ = this->create_wall_timer(std::chrono::seconds(SAMPLING_INTERVAL), topics_callback);

    std::function<void ()> consume_callback = std::bind(&mqtt_alive_publisher::consume_message, this, start_recording_topic, stop_recording_topic, stopped_recording_topic, started_recording_topic, bag_path.str());
    consume_timer_ = this->create_wall_timer(std::chrono::seconds(5), consume_callback);

    std::function<void ()> recording_callback = std::bind(&mqtt_alive_publisher::recording_status, this, bag_path.str(), recording_topic);
    recording_timer_ = this->create_wall_timer(std::chrono::seconds(RECORDING_STATUS_INTERVAL), recording_callback);

    // client_.subscribe(start_recording_topic, 0, subOpts, props);
    // client_.subscribe(stop_recording_topic, 0, subOpts, props);
    // callback cb(client_, start_recording_topic, stop_recording_topic, stopped_recording_topic);
    // client_.set_callback(cb);
    client_.subscribe(start_recording_topic, 0, subOpts, props);
    client_.subscribe(stop_recording_topic, 0, subOpts, props);

    client_.start_consuming();
  }

  ~mqtt_alive_publisher()
  {
    client_.disconnect();
  }

private:
  void timer_callback(std::string topic)
  {
    RCLCPP_INFO(get_logger(), "here");
    try {
        const char* payload = "1";
        mqtt::message_ptr msg = mqtt::make_message(topic, payload);
        client_.publish(msg);
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(get_logger(), exc.what());
    }
  }

  std::string fetch_executables()
  {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("ros2 pkg executables", "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    // std::string result = command("ros2 pkg executables");
    std::istringstream ss(result);
    std::string line;
    std::map<std::string, std::vector<std::string>> executables;
    while (getline(ss, line)) {
        auto pos = line.find(" ");
        std::string pkg = line.substr(0, pos);
        std::string exec = line.substr(pos+1, line.length());
        auto it = executables.find(pkg);
        if (it == executables.end()) {
          std::vector<std::string> execs = { exec };
          executables.insert(std::make_pair(pkg, execs));
        } else {
          std::vector<std::string> execs = it->second;
          execs.push_back(exec);
        }
    }
    return nlohmann::json(executables).dump();
  }

  void update_nodes(std::string topic)
  {
    std::vector<std::string> running_nodes = get_node_names();
    for (std::string node_name : running_nodes) {
      bool seen_before = false;
      for (std::string existing_node_name : all_nodes_) {
        if (existing_node_name.compare(node_name) == 0) {
          seen_before = true;
          break;
        }
      }
      if (!seen_before) {
        all_nodes_.push_back(node_name);
      }
    }

    std::vector<std::string> stopped_nodes;
    for (std::string node_name : all_nodes_) {
      bool running = false;
      for (std::string running_node : running_nodes) {
        if (running_node.compare(node_name) == 0) {
          running = true;
        }
      }
      if (!running) {
        stopped_nodes.push_back(node_name);
      }
    }

    std::vector<std::map<std::string, std::string>> nodes;
    for (std::string node_name : running_nodes) {
      std::map<std::string, std::string> node;
      node.insert(std::make_pair("name", node_name));
      node.insert(std::make_pair("status", "running"));
      nodes.push_back(node);
    }
    for (std::string node_name : stopped_nodes) {
      std::map<std::string, std::string> node;
      node.insert(std::make_pair("name", node_name));
      node.insert(std::make_pair("status", "stopped"));
      nodes.push_back(node);
    }

    try {
        std::string payload = nlohmann::json(nodes).dump();
        mqtt::message_ptr msg = mqtt::make_message(topic, payload.c_str());
        client_.publish(msg);
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(get_logger(), exc.what());
    }
  }

  std::string mqtt_topic(int id, std::string data)
  {
    std::stringstream stream;
    stream << "/";
    stream << id;
    stream << "/";
    stream << data;
    return stream.str();
  }

  void discover_topics()
  {
    std::map<std::string, std::vector<std::string>> topic_names_and_types = get_topic_names_and_types();
    
    for (auto it1 = topic_names_and_types.begin(); it1 != topic_names_and_types.end(); ++it1) {
      bool known = false;
      for (auto it2 = subscription_data_.begin(); it2 != subscription_data_.end(); ++it2) {
        if (it1->first.compare(it2->first) == 0) {
          known = true;
        }
      }
      if (!known) {
        // subscription config = {.name = it1->first, .types = it1->second, .message_count = 0};
        // subscription* ptr = &config;
        subscription * config = new subscription{.name = it1->first, .types = it1->second, .message_count = 0};

        rclcpp::SubscriptionBase::SharedPtr sub = nullptr;
        for (std::string type : it1->second) {
          if (type.compare("rcl_interfaces/msg/ParameterEvent") == 0) {
            std::function<void (rcl_interfaces::msg::ParameterEvent::SharedPtr)> func = 
              [config](const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) -> void { config->message_count++; };
            sub = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(it1->first, 10, func);
            break;
          } else if (type.compare("std_msgs/msg/String") == 0) {
            std::function<void (std_msgs::msg::String::SharedPtr)> func = 
              [config](const std_msgs::msg::String::SharedPtr msg) -> void { config->message_count++; };
            sub = this->create_subscription<std_msgs::msg::String>(it1->first, 10, func);
            break;
          }
        }
        subscription_data_.insert(std::make_pair(it1->first, config));
        if (sub != nullptr) {
          subscriptions_.insert(std::make_pair(it1->first, sub));
        }
      }
    }
  }

  void topics(std::string mqtt_topic)
  {
    std::vector<std::map<std::string, std::string>> topic_data;
    for (auto it = subscription_data_.begin(); it != subscription_data_.end(); ++it) {
      subscription* config = it->second;

      double frequency = config->message_count / SAMPLING_INTERVAL;
      config->message_count = 0;
      
      std::map<std::string, std::string> info;
      info.insert(std::make_pair("name", config->name));
      info.insert(std::make_pair("type", config->types[0]));
      info.insert(std::make_pair("frequency", std::to_string(frequency)));
      topic_data.push_back(info);
    }
    std::string payload = nlohmann::json(topic_data).dump();
    try {
        mqtt::message_ptr msg = mqtt::make_message(mqtt_topic, payload.c_str());
        client_.publish(msg);
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(get_logger(), exc.what());
    }
  }

  void consume_message(std::string start_topic, std::string stop_topic, std::string stopped_topic, std::string started_topic, std::string bag_path)
  {
    mqtt::const_message_ptr msg;
    bool has_msg = client_.try_consume_message(&msg);
    if (has_msg) {
      if (msg->get_topic() == start_topic) {
        bag_recording_pid_ = fork();
        if (bag_recording_pid_ < 0) {
          RCLCPP_ERROR(get_logger(), "error forking process");
        } else if (bag_recording_pid_ == 0) {
          std::string full_bag_path = bag_path + bag_recording_id_;
          std::string payload = msg->to_string();
          nlohmann::json data = nlohmann::json::parse(payload);
          bag_recording_id_ = std::to_string(data["id"].get<int>());
          nlohmann::json topics = data["topics"];
          int num_args = topics.size() + 6;
          char* args[num_args] = {"ros2", "bag", "record"};
          int i = 3;
          for (auto& topic : topics) {
            std::string str_name = topic["name"].get<std::string>();
            const char* name = str_name.c_str();
            args[i] = (char*) malloc(strlen(name)+1);
            strcpy(args[i], name);
            i++;
          }
          args[num_args-3] = "-o";
          args[num_args-2] = const_cast<char*>(full_bag_path.c_str());
          args[num_args-1] = NULL;
          execvp("ros2", args);
        } else if (bag_recording_pid_ > 0) {
          recording_ = true;
          try {
              nlohmann::json message_data;
              message_data["id"] = bag_recording_id_;
              mqtt::message_ptr msg = mqtt::make_message(started_topic, message_data.dump());
              client_.publish(msg);
          } catch (const mqtt::exception& exc) {
              RCLCPP_ERROR(get_logger(), exc.what());
          }
        }
      } else if (msg->get_topic() == stop_topic) {
        if (bag_recording_pid_ > 0) {
          kill(bag_recording_pid_, SIGINT);
          RCLCPP_INFO(get_logger(), "Stopped recording");
          sleep(10);
          std::string full_bag_path = bag_path + bag_recording_id_;
          std::uintmax_t size = directory_size(full_bag_path);
          std::ifstream t(full_bag_path + "/metadata.yaml");
          std::stringstream buffer;
          buffer << t.rdbuf();
          std::string metadata = buffer.str();
          nlohmann::json message_data;
          message_data["yaml"] = metadata;
          message_data["id"] = bag_recording_id_;
          message_data["size"] = size;
          try {
              mqtt::message_ptr msg = mqtt::make_message(stopped_topic, message_data.dump());
              client_.publish(msg);
          } catch (const mqtt::exception& exc) {
              RCLCPP_ERROR(get_logger(), exc.what());
          }
          recording_ = false;
        }
      }
    }
    else if (!client_.is_connected()) {
      while (!client_.is_connected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
    RCLCPP_INFO(get_logger(), "issue");
  }

  void recording_status(std::string bag_path, std::string started_topic)
  {
    if (recording_) {
      try {
        std::string full_bag_path = bag_path + bag_recording_id_;
        std::uintmax_t size = directory_size(full_bag_path);
        double rate = (size - previous_size_) / RECORDING_STATUS_INTERVAL;
        previous_size_ = size;
        nlohmann::json message_data;
        message_data["size"] = size;
        message_data["rate"] = rate;
        message_data["id"] = bag_recording_id_;
        mqtt::message_ptr msg = mqtt::make_message(started_topic, message_data.dump());
        client_.publish(msg);
      } catch(std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(get_logger(), e.what());
      } catch (mqtt::exception& e) {
        RCLCPP_ERROR(get_logger(), e.what());
      }
    }
  }

  std::uintmax_t directory_size(std::string path)
  {
    std::uintmax_t size = 0;
    for (const std::filesystem::directory_entry& f : std::filesystem::recursive_directory_iterator(path)) {
      if (std::filesystem::is_regular_file(f.path())) {
        size += std::filesystem::file_size(f.path());
      }
    }
    return size;
  }

  // std::string command(std::string cmd)
  // {
  //   std::array<char, 128> buffer;
  //   FILE* pipe(popen(cmd.c_str(), "r"), pclose);
  // }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr nodes_timer_;
  rclcpp::TimerBase::SharedPtr topic_discovery_timer_;
  rclcpp::TimerBase::SharedPtr topics_timer_;
  rclcpp::TimerBase::SharedPtr consume_timer_;
  rclcpp::TimerBase::SharedPtr recording_timer_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, subscription*> subscription_data_;
  mqtt::async_client client_;
  std::vector<std::string> all_nodes_;
  int bag_recording_pid_;
  std::string bag_recording_id_;
  std::uintmax_t previous_size_ = 0;
  bool recording_ = false;

  const int SAMPLING_INTERVAL = 15;
  const int RECORDING_STATUS_INTERVAL = 1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mqtt_alive_publisher>());
  rclcpp::shutdown();
  return 0;
}
