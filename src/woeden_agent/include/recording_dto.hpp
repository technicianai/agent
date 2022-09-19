#ifndef RECORDING_DTO_H
#define RECORDING_DTO_H

#include <string>

#include <nlohmann/json.hpp>

namespace woeden
{
struct recording_topic
{
  std::string name;
  std::string type;
  bool throttle;
  double frequency;

  static recording_topic from_json(nlohmann::json data);
  nlohmann::json to_json();
};

struct recording_status
{
  double eta;
  double rate;
  uintmax_t size;
  uint32_t trigger_id;
};

struct recording_metadata
{
  std::string metadata;
  uintmax_t size;
  uint32_t trigger_id;
};
}

#endif
