#ifndef UTILS_H
#define UTILS_H

#include "disk_monitor.hpp"
#include "recording_trigger.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

#include <uuid/uuid.h>

namespace woeden
{
std::string blocking_cmd(const char* cmd);

std::string uuid();
}

#endif
