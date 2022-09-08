#ifndef RECORDING_TRIGGER_H
#define RECORDING_TRIGGER_H

#include <nlohmann/json.hpp>

#include <string>

using namespace std;

namespace woeden
{
class recording_trigger
{
public:
  recording_trigger(uint32_t id, string comparison_topic, string comparison_type,
    string comparison_field, string comparison_value, string comparison_value_type);

  bool in(nlohmann::json data);
  string get_value(nlohmann::json data);
  string get_topic();
  
  bool evaluate(string value);

  nlohmann::json to_json();
  static recording_trigger from_json(nlohmann::json rt_json);

private:
  uint32_t id_;
  string comparison_topic_;
  string comparison_type_;
  string comparison_field_;
  string comparison_value_;
  string comparison_value_type_;
};
}

#endif
