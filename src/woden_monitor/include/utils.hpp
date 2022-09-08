#ifndef UTILS_H
#define UTILS_H

#include "disk_monitor.hpp"
#include "recording_trigger.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using namespace std;

namespace woeden
{
string blocking_cmd(const char* cmd);

void upload_bag(uint32_t bag_id, string base_path, vector<string> urls, function<void (string)> callback);

nlohmann::json mounts_to_json(vector<mount> mounts);
}

#endif
