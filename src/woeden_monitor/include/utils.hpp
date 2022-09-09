#ifndef UTILS_H
#define UTILS_H

#include "disk_monitor.hpp"
#include "recording_trigger.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace woeden
{
std::string blocking_cmd(const char* cmd);
}

#endif
