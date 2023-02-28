#ifndef STATUS_COMPARISON_H
#define STATUS_COMPARISON_H

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "key_value_comparison.hpp"

namespace woeden
{
class status_comparison
{
public:
  status_comparison(
    std::string name, std::vector<uint8_t> levels, std::vector<key_value_comparison *> key_values);
  bool evaluate(diagnostic_msgs::msg::DiagnosticStatus diagnostic_status);

  nlohmann::json to_json();
  static status_comparison * from_json(nlohmann::json data);

private:
  std::string name_;
  std::vector<uint8_t> levels_;
  std::vector<key_value_comparison *> key_values_;
};
}  // namespace woeden

#endif
