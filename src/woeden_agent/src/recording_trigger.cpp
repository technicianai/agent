#include "recording_trigger.hpp"

#include <string>

using namespace std;

namespace woeden
{
recording_trigger::recording_trigger(uint32_t id, string comparison_topic, string comparison_type,
  string comparison_field, string comparison_value, string comparison_value_type)
{
  id_ = id;
  comparison_topic_ = comparison_topic;
  comparison_type_ = comparison_type;
  comparison_field_ = comparison_field;
  comparison_value_ = comparison_value;
  comparison_value_type_ = comparison_value_type;
}

bool recording_trigger::in(nlohmann::json data)
{
  return data.contains(comparison_field_);
}

string recording_trigger::get_value(nlohmann::json data)
{
  return data[comparison_field_].get<string>();
}

string recording_trigger::get_topic()
{
  return comparison_topic_;
}

bool recording_trigger::evaluate(string value)
{
  if (comparison_value_type_ == "STRING") {
    return value == comparison_value_;
  } else {
    double val = stod(value);
    double comp = stod(comparison_value_);
    if (comparison_type_ == "EQUAL_TO") {
      return abs(val - comp) < 0.1;
    } else if (comparison_type_ == "GREATER_THAN") {
      return val > comp;
    } else if (comparison_type_ == "LESS_THAN") {
      return val < comp;
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
  return rt_json;
}

recording_trigger recording_trigger::from_json(nlohmann::json rt_json)
{
  return recording_trigger(
    rt_json["id"].get<uint64_t>(),
    rt_json["comparison_topic"].get<string>(),
    rt_json["comparison_type"].get<string>(),
    rt_json["comparison_field"].get<string>(),
    rt_json["comparison_value"].get<string>(),
    rt_json["comparison_value_type"].get<string>()
  );
}
}
