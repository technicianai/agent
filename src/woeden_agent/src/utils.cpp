#include "utils.hpp"

#include <array>
#include <cstdio>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

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

string uuid()
{
  uuid_t new_uuid;
  uuid_generate(new_uuid);
  char uuid_cstr[100];
  uuid_unparse(new_uuid, uuid_cstr);
  return string(uuid_cstr);
}
}
