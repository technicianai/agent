#ifndef config_H
#define config_H

#include "recording_trigger.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using namespace std;

namespace woeden
{
class config
{
public:
  config(string home_dir);

  uint32_t get_id();
  string get_password();
  vector<recording_trigger> get_recording_triggers();

  void add_recording_trigger(recording_trigger rt);

private:
  nlohmann::json load(string path);
  void save(nlohmann::json contents, string path);

  string path_;
  nlohmann::json contents_;
};
}

#endif
