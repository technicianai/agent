#ifndef RECORDING_DTO_H
#define RECORDING_DTO_H

#include <string>

using namespace std;

namespace woeden
{
struct recording_topic
{
  string name;
  bool throttle;
  double frequency;
};

struct recording_status
{
  double eta;
  double rate;
  uintmax_t size;
};

struct recording_metadata
{
  string metadata;
  uintmax_t size;
};
}

#endif
