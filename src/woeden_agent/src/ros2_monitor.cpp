#include "ros2_monitor.hpp"
#include "utils.hpp"

#include <chrono>
#include <sstream>

using namespace std;
using namespace placeholders;

namespace woeden
{
ros2_monitor::ros2_monitor(shared_ptr<mqtt_facade> facade, vector<recording_trigger> triggers) : Node("woeden_ros2_monitor"), facade_(facade), unassigned_triggers_(triggers)
{
  discover_packages();

  function<void ()> alive_cb = bind(&mqtt_facade::publish_alive, facade_);
  alive_timer_ = this->create_wall_timer(chrono::seconds(15), alive_cb);

  function<void ()> nodes_cb = bind(&ros2_monitor::discover_nodes, this);
  nodes_timer_ = this->create_wall_timer(chrono::seconds(30), nodes_cb);

  function<void ()> topics_cb = bind(&ros2_monitor::discover_topics, this);
  topics_timer_ = this->create_wall_timer(chrono::seconds(15), topics_cb);

  function<void ()> topics_freq_cb = bind(&ros2_monitor::sample_topic_freqs, this);
  topics_freq_timer_ = this->create_wall_timer(chrono::seconds(SAMPLING_INTERVAL), topics_freq_cb);
}

void ros2_monitor::add_trigger(recording_trigger rt)
{
  unassigned_triggers_.push_back(rt);
}

void ros2_monitor::discover_packages()
{
  robot_config rc;
  string output = blocking_cmd("ros2 pkg executables");
  istringstream ss(output);
  string line;
  while (getline(ss, line)) {
    auto pos = line.find(" ");

    string pkg_name = line.substr(0, pos);
    string exec = line.substr(pos+1, line.length());

    bool known_pkg = false;
    for (package& pkg : rc.packages) {
      if (pkg.name == pkg_name) {
        known_pkg = true;
        pkg.executables.push_back(exec);
      }
    }
    if (!known_pkg) {
      rc.packages.push_back({
        .name = pkg_name,
        .executables = { exec }
      });
    }
  }
  facade_->publish_robot_config(rc);
}

void ros2_monitor::discover_nodes()
{
  vector<string> node_names = get_node_names();

  for (string& node_name : node_names) {
    bool new_node = true;
    for (node& n : nodes_) {
      if (n.name == node_name) {
        new_node = false;
        n.status = "running";
        break;
      }
    }
    if (new_node) {
      nodes_.push_back({ .name = node_name, .status = "running" });
    }
  }

  for (node& n : nodes_) {
    bool stopped_node = true;
    for (string& node_name : node_names) {
      if (n.name == node_name) {
        stopped_node = false;
        break;
      }
    }
    if (stopped_node) {
      n.status = "stopped";
    }
  }

  facade_->publish_nodes(nodes_);
}

void ros2_monitor::discover_topics()
{
  map<string, vector<string>> topic_names_and_types = get_topic_names_and_types();
  for (auto it = topic_names_and_types.begin(); it != topic_names_and_types.end(); ++it) {
    string topic_name = it->first;
    string topic_type = it->second[0];
    
    vector<recording_trigger> new_topic_triggers;
    for (int i = unassigned_triggers_.size()-1; i >= 0; i--) {
      recording_trigger rt = unassigned_triggers_[i];
      if (topic_name == rt.get_topic()) {
        new_topic_triggers.push_back(rt);
        unassigned_triggers_.erase(unassigned_triggers_.begin()+i);
      }
    }

    bool known = false;
    for (topic* t : topics_) {
      if (topic_name == t->name) {
        vector<recording_trigger> topic_triggers = t->triggers;
        for (recording_trigger& rt : new_topic_triggers) {
          t->triggers.push_back(rt);
        }
        known = true;
      }
    }
    if (known) continue;

    topic* t = new topic{.name = topic_name, .type = topic_type, .message_count = 0, .triggers = new_topic_triggers};
    topics_.push_back(t);

    rclcpp::SubscriptionBase::SharedPtr sub = nullptr;
    if (topic_type == "std_msgs/msg/String") {
      function<void (shared_ptr<std_msgs::msg::String>)> cb = bind(&ros2_monitor::trigger_callback, this, _1, t);
      sub = create_subscription<std_msgs::msg::String>(topic_name, 10, cb);
    } else {
      function<void (shared_ptr<rclcpp::SerializedMessage>)> cb = bind(&ros2_monitor::default_callback, this, _1, t);
      sub = create_generic_subscription(topic_name, topic_type, 10, cb);
    }
    subscriptions_.push_back(sub);
  }
}

void ros2_monitor::sample_topic_freqs()
{
  vector<topic> data;
  for (topic* t : topics_) {
    double frequency = (double) t->message_count / (double) SAMPLING_INTERVAL;
    t->message_count = 0;
    t->frequency = frequency;
    data.push_back(*t);
  }
  facade_->publish_topics(data);
}

void ros2_monitor::default_callback(shared_ptr<rclcpp::SerializedMessage> msg, topic* t)
{
  t->message_count++;
}

void ros2_monitor::trigger_callback(shared_ptr<std_msgs::msg::String> msg, topic* t)
{
  t->message_count++;
  for (recording_trigger rt : t->triggers) {
    nlohmann::json msg_data = nlohmann::json::parse(msg->data);
    if (rt.in(msg_data) && rt.evaluate(msg_data)) {
      facade_->publish_autostart(rt.get_id());
    }
  }
}
}
