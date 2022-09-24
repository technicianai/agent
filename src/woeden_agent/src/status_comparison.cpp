#include "status_comparison.hpp"

#include <iostream>
#include <string>

using namespace std;

namespace woeden
{
status_comparison::status_comparison(string name, vector<uint8_t> levels, vector<key_value_comparison*> key_values)
{
  name_ = name;
  levels_ = levels;
  key_values_ = key_values;
}

bool status_comparison::evaluate(diagnostic_msgs::msg::DiagnosticStatus diagnostic_status)
{
  if (name_ == diagnostic_status.name) {
    for (uint8_t level : levels_) {
      if (level == diagnostic_status.level) {
        return true;
      }
    }
    for (key_value_comparison* kvc : key_values_) {
      for (diagnostic_msgs::msg::KeyValue kv : diagnostic_status.values) {
        if (kvc->evaluate(kv)) {
          return true;
        }
      }
    }
  }
  return false;
}

nlohmann::json status_comparison::to_json()
{
  nlohmann::json key_values;
  for (key_value_comparison* kvc : key_values_) {
    key_values.push_back(kvc->to_json());
  }
  nlohmann::json json;
  json["name"] = name_;
  json["levels"] = nlohmann::json(levels_);
  json["key_values"] = key_values;
  return json;
}

status_comparison* status_comparison::from_json(nlohmann::json data)
{
  vector<uint8_t> levels;
  for (uint8_t level : data["levels"]) {
    levels.push_back(level);
  }

  vector<key_value_comparison*> key_values;
  for (nlohmann::json kvc : data["key_values"]) {
    key_values.push_back(key_value_comparison::from_json(kvc));
  }

  return new status_comparison(
    data["name"].get<string>(),
    levels,
    key_values
  );
}
}
