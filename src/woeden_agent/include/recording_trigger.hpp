#ifndef RECORDING_TRIGGER_H
#define RECORDING_TRIGGER_H

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "key_value_comparison.hpp"
#include "recording_dto.hpp"
#include "status_array_comparison.hpp"
#include "status_comparison.hpp"

namespace woeden
{
class recording_trigger
{
public:
  recording_trigger(
    uint32_t id, std::string topic, std::string type, bool enabled,
    std::vector<recording_topic> record_topics, uint32_t duration, std::string base_path,
    std::string msgdef);

  uint32_t get_id();
  std::string get_topic();
  std::string get_topic_type();
  std::vector<recording_topic> get_record_topics();
  uint32_t get_duration();
  std::string get_base_path();
  bool is_enabled();
  std::string get_msgdef();

  void set_key_value_comparison(key_value_comparison * kvc);
  void set_status_comparison(status_comparison * sc);
  void set_status_array_comparison(status_array_comparison * sac);
  void set_enabled(bool enabled);

  bool evaluate(nlohmann::json data);
  bool evaluate(std::shared_ptr<diagnostic_msgs::msg::KeyValue> data);
  bool evaluate(std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> data);
  bool evaluate(std::shared_ptr<diagnostic_msgs::msg::DiagnosticArray> data);

  nlohmann::json to_reduced_json();
  nlohmann::json to_json();
  static recording_trigger from_json(nlohmann::json rt_json);

private:
  uint32_t id_;
  std::string topic_;
  std::string type_;
  key_value_comparison * key_value_comparison_ = nullptr;
  status_comparison * status_comparison_ = nullptr;
  status_array_comparison * status_array_comparison_ = nullptr;
  bool enabled_;
  std::vector<recording_topic> record_topics_;
  uint32_t duration_;
  std::string base_path_;
  std::string msgdef_;
};
}  // namespace woeden

#endif
