#ifndef DISK_DTO_H
#define DISK_DTO_H

#include <string>

namespace woeden
{
struct mount
{
  std::string path;
  long available;
  long total;
};
}

#endif
