#include "recording_trigger.hpp"

#include <string>

using namespace std;

namespace woeden
{
recording_trigger::recording_trigger(uint32_t id, string topic, string type,
  bool enabled, vector<recording_topic> record_topics, uint32_t duration, string base_path)
{
  id_ = id;
  topic_ = topic;
  type_ = type;
  enabled_ = enabled;
  record_topics_ = record_topics;
  duration_ = duration;
  base_path_ = base_path;
}

uint32_t recording_trigger::get_id()
{
  return id_;
}

string recording_trigger::get_topic()
{
  return topic_;
}

string recording_trigger::get_topic_type()
{
  return type_;
}

vector<recording_topic> recording_trigger::get_record_topics()
{
  return record_topics_;
}

uint32_t recording_trigger::get_duration()
{
  return duration_;
}

string recording_trigger::get_base_path()
{
  return base_path_;
}

bool recording_trigger::is_enabled()
{
  return enabled_;
}

void recording_trigger::set_key_value_comparison(key_value_comparison* kvc)
{
  key_value_comparison_ = kvc;
}

void recording_trigger::set_status_comparison(status_comparison* sc)
{
  status_comparison_ = sc;
}

void recording_trigger::set_status_array_comparison(status_array_comparison* sac)
{
  status_array_comparison_ = sac;
}

void recording_trigger::set_enabled(bool enabled)
{
  enabled_ = enabled;
}

bool recording_trigger::evaluate(nlohmann::json data)
{
  return key_value_comparison_->evaluate(data);
}

bool recording_trigger::evaluate(shared_ptr<diagnostic_msgs::msg::KeyValue> data)
{
  return key_value_comparison_->evaluate(*data);
}

bool recording_trigger::evaluate(shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> data)
{
  return status_comparison_->evaluate(*data);
}

bool recording_trigger::evaluate(shared_ptr<diagnostic_msgs::msg::DiagnosticArray> data)
{
  return status_array_comparison_->evaluate(*data);
}

nlohmann::json recording_trigger::to_reduced_json()
{
  nlohmann::json json;
  json["id"] = id_;
  json["enabled"] = enabled_;
  return json;
}

nlohmann::json recording_trigger::to_json()
{
  nlohmann::json rt_json;
  rt_json["id"] = id_;
  rt_json["topic"] = topic_;
  rt_json["type"] = type_;
  rt_json["enabled"] = enabled_;
  rt_json["duration"] = duration_;
  rt_json["base_path"] = base_path_;

  nlohmann::json topics;
  for (recording_topic& record_topic : record_topics_) {
    topics.push_back(record_topic.to_json());  
  }
  rt_json["topics"] = topics;

  if (type_ == "std_msgs/msg/String" || type_ == "diagnostic_msgs/msg/KeyValue") {
    rt_json["comparison"] = key_value_comparison_->to_json();
  } else if (type_ == "diagnostic_msgs/msg/DiagnosticStatus") {
    rt_json["comparison"] = status_comparison_->to_json();
  } else if (type_ == "diagnostic_msgs/msg/DiagnosticArray") {
    rt_json["comparison"] = status_array_comparison_->to_json();
  }

  return rt_json;
}

recording_trigger recording_trigger::from_json(nlohmann::json rt_json)
{
  vector<recording_topic> record_topics;
  for (auto& record_topic_json : rt_json["topics"]) {
    record_topics.push_back(recording_topic::from_json(record_topic_json));
  }

  string type = rt_json["type"].get<string>();

  recording_trigger rt = recording_trigger(
    rt_json["id"].get<uint64_t>(),
    rt_json["topic"].get<string>(),
    type,
    rt_json["enabled"].get<bool>(),
    record_topics,
    rt_json["duration"].get<uint32_t>(),
    rt_json["base_path"].get<string>()
  );

  nlohmann::json comp = rt_json["comparison"];
  if (type == "std_msgs/msg/String" || type == "diagnostic_msgs/msg/KeyValue") {
    key_value_comparison* kvc = key_value_comparison::from_json(comp);
    rt.set_key_value_comparison(kvc);
  } else if (type == "diagnostic_msgs/msg/DiagnosticStatus") {
    status_comparison* sc = status_comparison::from_json(comp);
    rt.set_status_comparison(sc);
  } else if (type == "diagnostic_msgs/msg/DiagnosticArray") {
    status_array_comparison* sac = status_array_comparison::from_json(comp);
    rt.set_status_array_comparison(sac);
  }

  return rt;
}
}
