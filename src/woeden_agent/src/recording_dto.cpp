#include "recording_dto.hpp"

using namespace std;

namespace woeden
{
recording_topic recording_topic::from_json(nlohmann::json data)
{
  return {
    .name = data["name"].get<string>(),
    .type = data["type"].get<string>(),
    .throttle = data["max_frequency"].get<bool>(),
    .frequency = data["frequency"].get<double>()};
}

nlohmann::json recording_topic::to_json()
{
  nlohmann::json data;
  data["name"] = name;
  data["type"] = type;
  data["max_frequency"] = throttle;
  data["frequency"] = frequency;
  return data;
}
}  // namespace woeden
