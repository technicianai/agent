#ifndef RECORDING_TRIGGER_H
#define RECORDING_TRIGGER_H

#include <nlohmann/json.hpp>

#include <string>

namespace woeden
{
class recording_trigger
{
public:
  recording_trigger(uint32_t id, std::string comparison_topic, std::string comparison_type,
    std::string comparison_field, std::string comparison_value, std::string comparison_value_type);

  bool in(nlohmann::json data);
  std::string get_value(nlohmann::json data);
  std::string get_topic();
  
  bool evaluate(std::string value);

  nlohmann::json to_json();
  static recording_trigger from_json(nlohmann::json rt_json);

private:
  uint32_t id_;
  std::string comparison_topic_;
  std::string comparison_type_;
  std::string comparison_field_;
  std::string comparison_value_;
  std::string comparison_value_type_;
};
}

#endif
