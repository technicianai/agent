#include "disk_monitor.hpp"
#include "utils.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <string>

#include <unistd.h>
#include <sys/stat.h>

using namespace std;

namespace woeden
{
disk_monitor::disk_monitor(shared_ptr<mqtt_facade> facade) : Node("woeden_disk_monitor"), facade_(facade)
{
  sample();
  function<void ()> sample = bind(&disk_monitor::sample, this);
  timer_ = create_wall_timer(chrono::seconds(SAMPLING_INTERVAL), sample);
}

vector<mount> disk_monitor::get_mounts()
{
  return mounts_;
}

long disk_monitor::remaining(string path)
{
  for (const mount & mnt : mounts_) {
    if (mnt.path == path) {
      return (long)((double) mnt.available - MINIMUM_DRIVE_SPACE_PERCENTAGE * (double) mnt.total);
    }
  }
  string error = "mount not found: " + path;
  throw logic_error(error.c_str());
}

void disk_monitor::sample()
{
  string output = blocking_cmd("df --output='target','avail','size' -B 1 | tail -n +2 | tr -s ' '");

  istringstream ss(output);
  string line;
  vector<mount> mounts;
  while (getline(ss, line)) {
    stringstream split(line);

    string path;
    getline(split, path, ' ');

    string avail_str;
    getline(split, avail_str, ' ');
    long available = stol(avail_str);

    string total_str;
    getline(split, total_str, ' ');
    long total = stol(total_str);

    struct stat path_stat;
    stat(path.c_str(), &path_stat);

    if (access(path.c_str(), W_OK) == 0 && !S_ISREG(path_stat.st_mode)) {
      mounts.push_back({
        .path = path,
        .available = available,
        .total = total
      });
    }
  }

  mounts_ = mounts;
  facade_->publish_mounted_paths(mounts_);
}
}
