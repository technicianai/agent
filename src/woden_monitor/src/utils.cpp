#include "utils.hpp"

#include <array>
#include <cstdio>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std;

namespace woeden
{
string blocking_cmd(const char* cmd)
{
  array<char, 128> buffer;
  string result;
  unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

void upload_bag(uint32_t bag_id, string base_path, vector<string> urls, function<void (string)> callback)
{
  string command = "python3 /woeden_monitor/bag_utils/upload.py ";
  command += to_string(bag_id) + " ";
  command += base_path + " ";
  for (const string & url : urls) {
    command += "\"" + url + "\" ";
  }

  auto f = [command, callback]() {
    string result = blocking_cmd(command.c_str());
    callback(result);
  };

  thread thread_object(f);
  thread_object.detach();
}

nlohmann::json mounts_to_json(vector<mount> mounts)
{
  nlohmann::json mapped_mounts;
  for (const mount & mnt : mounts) {
    nlohmann::json mapped_mount;

    mapped_mount["path"] = mnt.path;
    mapped_mount["available"] = mnt.available;
    mapped_mount["total"] = mnt.total;

    mapped_mounts.push_back(mapped_mount);
  }
  return mapped_mounts;
}
}
