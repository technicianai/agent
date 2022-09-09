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

void config::add_recording_trigger(recording_trigger rt)
{
  nlohmann::json rt_json = rt.to_json();
  contents_["triggers"].push_back(rt_json);
  save(contents_, path_);
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
}
