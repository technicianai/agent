#include "config.hpp"
#include "utils.hpp"

#include <fstream>

using namespace std;

namespace woeden
{
config::config(string home_dir)
{
  path_ = home_dir + "/woeden/config";
  contents_ = load(path_);
}

uint32_t config::get_id()
{
  return contents_["id"].get<uint32_t>();
}

string config::get_password()
{
  return contents_["password"].get<string>();
}

vector<recording_trigger> config::get_recording_triggers()
{
  vector<recording_trigger> rts;
  for (const auto& rt_json : contents_["triggers"]) {
    rts.push_back(recording_trigger::from_json(rt_json));
  }
  return rts;
}

void config::add_trigger(recording_trigger rt)
{
  nlohmann::json rt_json = rt.to_json();
  contents_["triggers"].push_back(rt_json);
  save(contents_, path_);
}

void config::update_trigger(uint32_t id, bool enabled)
{
  for (auto& trigger : contents_["triggers"]) {
    if (id == trigger["id"].get<uint32_t>()) {
      trigger["enabled"] = enabled;
      save(contents_, path_);
      break;
    }
  }
}

nlohmann::json config::load(string path)
{
  ifstream config_file;
  config_file.open(path);
  return nlohmann::json::parse(config_file);
}

void config::save(nlohmann::json contents, string path)
{
  ofstream file(path);
  file << contents.dump();
}

always_record_config config::get_always_record()
{
  if (!contents_.contains("always_record")) {
    return {
      .duration = 0,
      .enabled = false,
      .base_path = "/"
    };
  }
  return {
    .duration = contents_["always_record"]["duration"].get<uint32_t>(),
    .enabled = contents_["always_record"]["enabled"].get<bool>(),
    .base_path = contents_["always_record"]["base_path"]
  };
}

void config::update_always_record(uint32_t duration, bool enabled, string base_path)
{
  nlohmann::json json;
  json["duration"] = duration;
  json["enabled"] = enabled;
  json["base_path"] = base_path;
  contents_["always_record"] = json;
  save(contents_, path_);
}
}
