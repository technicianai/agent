#ifndef KEY_VALUE_COMPARISON_H
#define KEY_VALUE_COMPARISON_H

#include "diagnostic_msgs/msg/key_value.hpp"

#include <nlohmann/json.hpp>

#include <string>

namespace woeden
{
class key_value_comparison
{
public:
  key_value_comparison(std::string field, std::string comparator, std::string value, std::string value_type);
  bool evaluate(nlohmann::json data);
  bool evaluate(diagnostic_msgs::msg::KeyValue data);

  nlohmann::json to_json();
  static key_value_comparison* from_json(nlohmann::json data);

private:
  std::string field_;
  std::string comparator_;
  std::string value_;
  std::string value_type_;
};
}

#endif
