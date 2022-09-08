#ifndef ROS2_DTO_H
#define ROS2_DTO_H

#include <string>
#include <vector>

using namespace std;

namespace woeden
{
struct package
{
  string name;
  vector<string> executables;
};

struct robot_config
{
  vector<package> packages;
};

struct node
{
  string name;
  string status;
};

struct topic
{
  string name;
  string type;
  double frequency;
  uint32_t message_count;
  //vector<recording_trigger> triggers;
};
}

#endif
