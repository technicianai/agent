#ifndef DISK_DTO_H
#define DISK_DTO_H

#include <string>

using namespace std;

namespace woeden
{
struct mount
{
  string path;
  long available;
  long total;
};
}

#endif
