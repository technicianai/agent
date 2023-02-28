#ifndef CONFIG_H
#define CONFIG_H

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "recording_trigger.hpp"

namespace woeden
{
struct always_record_config
{
  uint32_t duration;
  bool enabled;
  std::string base_path;
};

class config
{
public:
  config(std::string home_dir);

  uint32_t get_id();
  std::string get_password();
  std::vector<recording_trigger> get_recording_triggers();
  always_record_config get_always_record();
  double get_max_bandwidth();

  void add_trigger(recording_trigger rt);
  void update_trigger(uint32_t id, bool enabled);
  void update_always_record(uint32_t duration, bool enabled, std::string base_path);
  void update_max_bandwidth(double bandwidth);

private:
  nlohmann::json load(std::string path);
  void save(nlohmann::json contents, std::string path);

  std::string path_;
  nlohmann::json contents_;
};
}  // namespace woeden

#endif
