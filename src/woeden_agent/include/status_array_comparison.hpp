#ifndef STATUS_ARRAY_COMPARISON_H
#define STATUS_ARRAY_COMPARISON_H

#include "status_comparison.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace woeden
{
class status_array_comparison
{
public:
  status_array_comparison(std::vector<status_comparison*> statuses);
  bool evaluate(diagnostic_msgs::msg::DiagnosticArray data);

  nlohmann::json to_json();
  static status_array_comparison* from_json(nlohmann::json data);

private:
  std::vector<status_comparison*> statuses_;
};
}

#endif
