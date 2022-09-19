#include "recording_trigger.hpp"

#include <string>

using namespace std;

namespace woeden
{
recording_trigger::recording_trigger(uint32_t id, string comparison_topic, string comparison_type,
  string comparison_field, string comparison_value, string comparison_value_type, string topic_type,
  bool enabled, std::vector<recording_topic> record_topics, uint32_t duration, string base_path)
{
  id_ = id;
  comparison_topic_ = comparison_topic;
  comparison_type_ = comparison_type;
  comparison_field_ = comparison_field;
  comparison_value_ = comparison_value;
  comparison_value_type_ = comparison_value_type;
  topic_type_ = topic_type;
  enabled_ = enabled;
  record_topics_ = record_topics;
  duration_ = duration;
  base_path_ = base_path;
}

bool recording_trigger::in(nlohmann::json data)
{
  return data.contains(comparison_field_);
}

bool recording_trigger::in(shared_ptr<diagnostic_msgs::msg::KeyValue> data)
{
  return data->key == comparison_field_;
}

uint32_t recording_trigger::get_id()
{
  return id_;
}

string recording_trigger::get_topic()
{
  return comparison_topic_;
}

string recording_trigger::get_topic_type()
{
  return topic_type_;
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

bool recording_trigger::evaluate(nlohmann::json data)
{
  if (comparison_value_type_ == "STRING") {
    string value = data[comparison_field_].get<string>();
    return value == comparison_value_;
  } else {
    double comp = stod(comparison_value_);
    double value = data[comparison_field_].get<double>();
    if (comparison_type_ == "EQUAL_TO") {
      return abs(value - comp) < 0.1;
    } else if (comparison_type_ == "GREATER_THAN") {
      return value > comp;
    } else if (comparison_type_ == "LESS_THAN") {
      return value < comp;
    }
  }
  return false;
}

bool recording_trigger::evaluate(shared_ptr<diagnostic_msgs::msg::KeyValue> data)
{
  if (comparison_value_type_ == "STRING") {
    return data->value == comparison_value_;
  } else {
    double comp = stod(comparison_value_);
    double value = stod(data->value);
    if (comparison_type_ == "EQUAL_TO") {
      return abs(value - comp) < 0.1;
    } else if (comparison_type_ == "GREATER_THAN") {
      return value > comp;
    } else if (comparison_type_ == "LESS_THAN") {
      return value < comp;
    }
  }
  return false;
}

nlohmann::json recording_trigger::to_json()
{
  nlohmann::json rt_json;
  rt_json["id"] = id_;
  rt_json["comparison_topic"] = comparison_topic_;
  rt_json["comparison_type"] = comparison_type_;
  rt_json["comparison_field"] = comparison_field_;
  rt_json["comparison_value"] = comparison_value_;
  rt_json["comparison_value_type"] = comparison_value_type_;
  rt_json["topic_type"] = topic_type_;
  rt_json["enabled"] = enabled_;
  nlohmann::json topics;
  for (recording_topic& record_topic : record_topics_) {
    topics.push_back(record_topic.to_json())
;  }
  rt_json["topics"] = topics;
  rt_json["duration"] = duration_;
  rt_json["base_path"] = base_path_;
  return rt_json;
}

recording_trigger recording_trigger::from_json(nlohmann::json rt_json)
{
  vector<recording_topic> record_topics;
  for (auto& record_topic_json : rt_json["topics"]) {
    record_topics.push_back(recording_topic::from_json(record_topic_json));
  }
  return recording_trigger(
    rt_json["id"].get<uint64_t>(),
    rt_json["comparison_topic"].get<string>(),
    rt_json["comparison_type"].get<string>(),
    rt_json["comparison_field"].get<string>(),
    rt_json["comparison_value"].get<string>(),
    rt_json["comparison_value_type"].get<string>(),
    rt_json["topic_type"].get<string>(),
    rt_json["enabled"].get<bool>(),
    record_topics,
    rt_json["duration"].get<uint32_t>(),
    rt_json["base_path"].get<string>()
  );
}
}
