#ifndef config_H
#define config_H

#include "recording_trigger.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace woeden
{
class config
{
public:
  config(std::string home_dir);

  uint32_t get_id();
  std::string get_password();
  std::vector<recording_trigger> get_recording_triggers();

  void add_trigger(recording_trigger rt);

private:
  nlohmann::json load(std::string path);
  void save(nlohmann::json contents, std::string path);

  std::string path_;
  nlohmann::json contents_;
};
}

#endif
