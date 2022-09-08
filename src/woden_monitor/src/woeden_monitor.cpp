#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <thread>
#include <filesystem>

#include <sqlite3.h> 
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_event.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mqtt/async_client.h"
#include "mqtt/connect_options.h"

#include "disk_monitor.hpp"
#include "recording_manager.hpp"
#include "recording_trigger.hpp"
#include "recording_dto.hpp"
#include "utils.hpp"
#include "config.hpp"
#include "mqtt_facade.hpp"
#include "ros2_monitor.hpp"

using namespace woeden;
using namespace std::placeholders;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string host = getenv("HOST");
  std::string home = getenv("HOME");

  config c(home);

  auto facade = std::make_shared<mqtt_facade>(host, c.get_id(), c.get_password());

  auto dm = std::make_shared<disk_monitor>(facade);
  auto rm = std::make_shared<recording_manager>(dm, facade);
  auto r2m = std::make_shared<ros2_monitor>(facade);

  facade->set_record_callback(std::bind(&recording_manager::start, rm, _1, _2, _3));
  facade->set_stop_callback(std::bind(&recording_manager::stop, rm));
  facade->set_upload_callback(std::bind(&recording_manager::upload, rm, _1, _2, _3));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dm);
  executor.add_node(rm);
  executor.add_node(r2m);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
