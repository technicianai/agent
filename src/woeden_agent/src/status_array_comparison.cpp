#include "status_array_comparison.hpp"

#include <string>

using namespace std;

namespace woeden
{
status_array_comparison::status_array_comparison(vector<status_comparison*> statuses)
{
  statuses_ = statuses;
}

bool status_array_comparison::evaluate(diagnostic_msgs::msg::DiagnosticArray data)
{
  for (status_comparison* sc : statuses_) {
    for (diagnostic_msgs::msg::DiagnosticStatus status : data.status) {
      if (sc->evaluate(status)) {
        return true;
      }
    }
  }
  return false;
}

nlohmann::json status_array_comparison::to_json()
{
  nlohmann::json json;
  for (status_comparison* sc : statuses_) {
    json.push_back(sc->to_json());
  }
  return json;
}

status_array_comparison* status_array_comparison::from_json(nlohmann::json data)
{
  vector<status_comparison*> statuses;
  for (nlohmann::json sc : data) {
    statuses.push_back(status_comparison::from_json(sc));
  }
  return new status_array_comparison(statuses);
}
}
