#ifndef UTILS_H
#define UTILS_H

#include <uuid/uuid.h>

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "disk_monitor.hpp"
#include "recording_trigger.hpp"

namespace woeden
{
std::string blocking_cmd(const char * cmd);

std::string uuid();
bool is_valid_uuid(std::string uuid);
}  // namespace woeden

#endif
