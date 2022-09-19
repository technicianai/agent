#ifndef RECORDING_TRIGGER_H
#define RECORDING_TRIGGER_H

#include "recording_dto.hpp"

#include <nlohmann/json.hpp>

#include "diagnostic_msgs/msg/key_value.hpp"

#include <string>
#include <vector>

namespace woeden
{
class recording_trigger
{
public:
  recording_trigger(uint32_t id, std::string comparison_topic, std::string comparison_type,
    std::string comparison_field, std::string comparison_value, std::string comparison_value_type,
    std::string topic_type, bool enabled, std::vector<recording_topic> record_topics, uint32_t duration,
    std::string base_path);

  bool in(nlohmann::json data);
  bool in(std::shared_ptr<diagnostic_msgs::msg::KeyValue> data);
  uint32_t get_id();
  std::string get_topic();
  std::string get_topic_type();
  std::vector<recording_topic> get_record_topics();
  uint32_t get_duration();
  std::string get_base_path();
  
  bool evaluate(nlohmann::json data);
  bool evaluate(std::shared_ptr<diagnostic_msgs::msg::KeyValue> data);

  nlohmann::json to_json();
  static recording_trigger from_json(nlohmann::json rt_json);

private:
  uint32_t id_;
  std::string comparison_topic_;
  std::string comparison_type_;
  std::string comparison_field_;
  std::string comparison_value_;
  std::string comparison_value_type_;
  std::string topic_type_;
  bool enabled_;
  std::vector<recording_topic> record_topics_;
  uint32_t duration_;
  std::string base_path_;
};
}

#endif
