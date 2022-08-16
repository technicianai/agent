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

struct subscription
{
  std::string name;
  std::vector<std::string> types;
  int message_count;
};

static int sql_callback(void *NotUsed, int argc, char **argv, char **azColName) {
   return 0;
}

class woeden_monitor : public rclcpp::Node
{
public:
  woeden_monitor() : Node("woeden_monitor"), client_("tcp://localhost:1883", "paho-cpp-data-publish", nullptr)
  {
    std::stringstream id_path;
    id_path << getenv("HOME");
    id_path << "/woeden/id";
    std::ifstream id_file;
    id_file.open(id_path.str());
    int id;
    id_file >> id;

    std::string execs_output = execute_command("ros2 pkg executables");
    nlohmann::json execs = parse_executables(execs_output);

    nlohmann::json config;
    config["executables"] = execs;

    std::stringstream bag_path;
    bag_path << getenv("HOME");
    bag_path << "/woeden/bags/";
    
    std::string alive_topic = mqtt_topic(id, "alive");
    std::string config_topic = mqtt_topic(id, "config");
    std::string nodes_topic = mqtt_topic(id, "nodes");
    std::string topics_topic = mqtt_topic(id, "topics");
    std::string mounted_paths_topic = mqtt_topic(id, "mounted_paths");
    std::string start_recording_topic = mqtt_topic(id, "record");
    std::string stop_recording_topic = mqtt_topic(id, "stop");
    std::string stopped_recording_topic = mqtt_topic(id, "stopped");
    std::string started_recording_topic = mqtt_topic(id, "started");
    std::string recording_topic = mqtt_topic(id, "recording");
    std::string upload_topic = mqtt_topic(id, "upload");
    std::string uploading_topic = mqtt_topic(id, "uploading");
    std::string uploaded_topic = mqtt_topic(id, "uploaded");

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

    sleep(5);
    mqtt::message_ptr msg = mqtt::make_message(config_topic, config.dump().c_str());
    client_.publish(msg);
    
    std::function<void ()> alive_callback = std::bind(&woeden_monitor::timer_callback, this, alive_topic);
    timer_ = this->create_wall_timer(std::chrono::seconds(15), alive_callback);

    std::function<void ()> nodes_callback = std::bind(&woeden_monitor::update_nodes, this, nodes_topic);
    nodes_timer_ = this->create_wall_timer(std::chrono::seconds(15), nodes_callback);

    std::function<void ()> mounted_paths_callback = std::bind(&woeden_monitor::update_mounts, this, mounted_paths_topic);
    mounted_paths_timer_ = this->create_wall_timer(std::chrono::seconds(5), mounted_paths_callback);

    std::function<void ()> topic_discovery_callback = std::bind(&woeden_monitor::discover_topics, this);
    topic_discovery_timer_ = this->create_wall_timer(std::chrono::seconds(15), topic_discovery_callback);

    std::function<void ()> topics_callback = std::bind(&woeden_monitor::topics, this, topics_topic);
    topics_timer_ = this->create_wall_timer(std::chrono::seconds(SAMPLING_INTERVAL), topics_callback);

    std::function<void ()> recording_callback = std::bind(&woeden_monitor::recording_status, this, bag_path.str(), recording_topic, stopped_recording_topic);
    recording_timer_ = this->create_wall_timer(std::chrono::seconds(RECORDING_STATUS_INTERVAL), recording_callback);

    while (!client_.is_connected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    try {
      client_.subscribe(start_recording_topic, 0, subOpts, props);
      client_.subscribe(stop_recording_topic, 0, subOpts, props);
      client_.subscribe(upload_topic, 0, subOpts, props);
      client_.start_consuming();
    } catch (mqtt::exception& e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }

    std::function<void ()> consume_callback = std::bind(&woeden_monitor::consume_message, this, start_recording_topic, stop_recording_topic, stopped_recording_topic, started_recording_topic, upload_topic, uploaded_topic, bag_path.str());
    consume_timer_ = this->create_wall_timer(std::chrono::seconds(5), consume_callback);

    std::cout << "Connection to Woeden was successful. Actively monitoring." << std::endl;
  }

  ~woeden_monitor()
  {
    client_.disconnect();
  }

private:
  void timer_callback(std::string topic)
  {
    try {
        const char* payload = "1";
        mqtt::message_ptr msg = mqtt::make_message(topic, payload);
        client_.publish(msg);
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(get_logger(), exc.what());
    }
  }

  std::string execute_command(const char* command)
  {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command, "r"), pclose);
    if (!pipe) {
      throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      result += buffer.data();
    }
    return result;
  }

  nlohmann::json parse_executables(std::string command_output)
  {
    std::istringstream ss(command_output);
    nlohmann::json executables;
    std::string line;
    while (getline(ss, line)) {
        auto pos = line.find(" ");
        std::string pkg = line.substr(0, pos);
        std::string exec = line.substr(pos+1, line.length());
        executables[pkg].push_back(exec);
    }
    return executables;
  }

  void update_mounts(std::string mounted_paths_topic)
  {
    std::string mounts_output = execute_command("df --output='target','avail','size' -B 1 | tail -n +2 | tr -s ' '");
    nlohmann::json mounts = parse_mounts(mounts_output);
    mqtt::message_ptr msg = mqtt::make_message(mounted_paths_topic, mounts.dump().c_str());
    client_.publish(msg);
    for (auto& mount : mounts) {
      std::string path = mount["path"].get<std::string>();
      long total = mount["total"].get<long>();
      long avail = mount["available"].get<long>();
      long remaining = (long)((double)avail - MINIMUM_DRIVE_SPACE_PERCENTAGE * (double)total);
      auto it = path_space_remaining_.find(path);
      if (it != path_space_remaining_.end()) {
        it->second = remaining;
      } else {
        path_space_remaining_.insert(std::make_pair(path, remaining));
      }
    }
  }

  nlohmann::json parse_mounts(std::string command_output)
  {
    std::istringstream ss(command_output);
    std::string line;
    nlohmann::json mounts;
    while (getline(ss, line)) {
      std::stringstream split(line);

      std::string path;
      std::getline(split, path, ' ');

      std::string avail_str;
      std::getline(split, avail_str, ' ');
      long avail = std::stol(avail_str);

      std::string total_str;
      std::getline(split, total_str, ' ');
      long total = std::stol(total_str);

      struct stat path_stat;
      stat(path.c_str(), &path_stat);

      if (access(path.c_str(), W_OK) == 0 && !S_ISREG(path_stat.st_mode)) {
        nlohmann::json mount;
        mount["path"] = path;
        mount["available"] = avail;
        mount["total"] = total;
        mounts.push_back(mount);
      }
    }
    return mounts;
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

    std::string payload = nlohmann::json(nodes).dump();
    if (previous_nodes_.empty() || previous_nodes_ != payload) {
      try {
          mqtt::message_ptr msg = mqtt::make_message(topic, payload.c_str());
          client_.publish(msg);
      } catch (const mqtt::exception& exc) {
          RCLCPP_ERROR(get_logger(), exc.what());
      }
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

      double frequency = (double) config->message_count / (double) SAMPLING_INTERVAL;
      config->message_count = 0;
      
      std::map<std::string, std::string> info;
      info.insert(std::make_pair("name", config->name));
      info.insert(std::make_pair("type", config->types[0]));
      info.insert(std::make_pair("frequency", std::to_string(frequency)));
      topic_data.push_back(info);
    }
    std::string payload = nlohmann::json(topic_data).dump();
    if (previous_topics_.empty() || previous_topics_ != payload) {
      try {
        mqtt::message_ptr msg = mqtt::make_message(mqtt_topic, payload.c_str());
        client_.publish(msg);
      } catch (const mqtt::exception& exc) {
          RCLCPP_ERROR(get_logger(), exc.what());
      }
      previous_topics_ = payload;
    }
  }

  void consume_message(std::string start_topic, std::string stop_topic, std::string stopped_topic, std::string started_topic, std::string upload_topic, std::string uploaded_topic, std::string bag_path)
  {
    mqtt::const_message_ptr msg;
    if (!client_.try_consume_message(&msg)) return;
    if (msg->get_topic() == start_topic) {
      std::string payload = msg->to_string();
      nlohmann::json data = nlohmann::json::parse(payload);
      bag_recording_id_ = std::to_string(data["id"].get<int>());
      current_bag_base_path_ = data["base_path"];
      current_bag_path_ = current_bag_base_path_ + "/woeden/bags/" + bag_recording_id_;

      nlohmann::json topics = data["topics"];
      for (auto& topic : topics) {
        if (!topic["max_frequency"].get<bool>()) {
          continue;
        }
        int pid = fork();
        if (pid == 0) {
          setsid();
          char* args[8];
          int i = 0;

          std::string str_name = topic["name"].get<std::string>();
          std::string freq = std::to_string(topic["frequency"].get<double>());

          args[i++] = "ros2";
          args[i++] = "run";
          args[i++] = "topic_tools";
          args[i++] = "throttle";
          args[i++] = "messages";

          const char* intopic = str_name.c_str();
          args[i] = (char*) malloc(strlen(intopic)+1);
          strcpy(args[i++], intopic);

          const char* msgs_per_sec = freq.c_str();
          args[i] = (char*) malloc(strlen(msgs_per_sec)+1);
          strcpy(args[i++], msgs_per_sec);

          std::string outtopic_str = str_name + "/throttle";
          const char* outtopic = outtopic_str.c_str();
          args[i] = (char*) malloc(strlen(outtopic)+1);
          strcpy(args[i++], outtopic);

          args[i] = NULL;

          execvp("ros2", args);
        } else if (pid > 0) {
          throttle_pids_.push_back(pid);
        }
      }

      bag_recording_pid_ = fork();
      if (bag_recording_pid_ < 0) {
        RCLCPP_ERROR(get_logger(), "error forking process");
      } else if (bag_recording_pid_ == 0) {
        char* args[topics.size() + 6] = { "ros2", "bag", "record" };
        int i = 3;
        for (auto& topic : topics) {
          std::string str_name = topic["name"].get<std::string>();
          if (topic["max_frequency"].get<bool>()) {
            str_name += "/throttle";
          }
          const char* name = str_name.c_str();
          args[i] = (char*) malloc(strlen(name)+1);
          strcpy(args[i++], name);
        }
        args[i++] = "-o";
        args[i++] = const_cast<char*>(current_bag_path_.c_str());
        args[i++] = NULL;

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
        end_recording(stopped_topic);
      }
    } else if (msg->get_topic() == upload_topic) {
      std::string payload = msg->to_string();
      nlohmann::json data = nlohmann::json::parse(payload);
      std::string base_path = data["base_path"];
      std::string str_id = std::to_string(data["id"].get<int>());
      nlohmann::json urls = data["urls"];
      std::string command = "python3 /woeden_monitor/bag_utils/upload.py " + str_id + " " + base_path + " ";
      for (auto& json_url : urls) {
        std::string str_url = "\"" + json_url.get<std::string>() + "\"";
        command += str_url + " ";
      }
      auto logger = get_logger();
      auto f = [command, uploaded_topic, this]() {
        std::string result = execute_command(command.c_str());
        mqtt::message_ptr msg = mqtt::make_message(uploaded_topic, result.c_str());
        this->client_.publish(msg);
      };
      std::thread thread_object(f);
      thread_object.detach();
    }
  }

  void recording_status(std::string bag_path, std::string started_topic, std::string stopped_topic)
  {
    if (recording_) {
      try {
        auto it = path_space_remaining_.find(current_bag_base_path_);
        if (it->second <= 0) {
          end_recording(stopped_topic);
          return;
        }

        std::uintmax_t size = directory_size(current_bag_path_);
        double rate = (size - previous_size_) / RECORDING_STATUS_INTERVAL;
        previous_size_ = size;
        nlohmann::json message_data;
        message_data["size"] = size;
        message_data["rate"] = rate;
        message_data["eta"] = (double) it->second / rate;
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

  void end_recording(std::string stopped_topic)
  {
    kill(bag_recording_pid_, SIGINT);
    for (int pid : throttle_pids_) {
      int pgid = getpgid(pid);
      killpg(pgid, SIGKILL);
    }
    sleep(10);

    // update topic names
    for (const auto & entry : std::filesystem::directory_iterator(current_bag_path_)) {
      std::string db_path = entry.path();
      if (db_path.find(".db3") != std::string::npos) {
        sqlite3 *db;
        int rc = sqlite3_open(db_path.c_str(), &db);
        char *zErrMsg = 0;
        rc = sqlite3_exec(db, "UPDATE topics SET name=replace(name, '/throttle', '');", sql_callback, 0, &zErrMsg);
        sqlite3_close(db);
      }
    }

    std::uintmax_t size = directory_size(current_bag_path_);

    // remove "/throttle" from metadata.yaml
    std::ifstream t(current_bag_path_ + "/metadata.yaml");
    std::stringstream buffer;
    buffer << t.rdbuf();
    std::string metadata = buffer.str();
    size_t pos = 0;
    while((pos = metadata.find("/throttle", pos)) != std::string::npos) {
      metadata.replace(pos, 9, "");
    }
    std::ofstream out(current_bag_path_ + "/metadata.yaml");
    out << metadata;
    out.close();

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

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr nodes_timer_;
  rclcpp::TimerBase::SharedPtr topic_discovery_timer_;
  rclcpp::TimerBase::SharedPtr topics_timer_;
  rclcpp::TimerBase::SharedPtr consume_timer_;
  rclcpp::TimerBase::SharedPtr recording_timer_;
  rclcpp::TimerBase::SharedPtr mounted_paths_timer_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, subscription*> subscription_data_;
  std::map<std::string, long> path_space_remaining_;
  mqtt::async_client client_;
  std::vector<std::string> all_nodes_;
  int bag_recording_pid_;
  int bag_upload_pid_;
  std::vector<int> throttle_pids_;
  std::string bag_recording_id_;
  std::string current_bag_path_;
  std::string previous_topics_;
  std::string previous_nodes_;
  std::string current_bag_base_path_;
  std::uintmax_t previous_size_ = 0;
  bool recording_ = false;

  const int SAMPLING_INTERVAL = 15;
  const int RECORDING_STATUS_INTERVAL = 1;
  const double MINIMUM_DRIVE_SPACE_PERCENTAGE = 0.03;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<woeden_monitor>());
  rclcpp::shutdown();
  return 0;
}
