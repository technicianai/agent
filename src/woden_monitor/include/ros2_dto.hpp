#ifndef ROS2_DTO_H
#define ROS2_DTO_H

#include <string>
#include <vector>

namespace woeden
{
struct package
{
  std::string name;
  std::vector<std::string> executables;
};

struct robot_config
{
  std::vector<package> packages;
};

struct node
{
  std::string name;
  std::string status;
};

struct topic
{
  std::string name;
  std::string type;
  double frequency;
  uint32_t message_count;
  //vector<recording_trigger> triggers;
};
}

#endif
