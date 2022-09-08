#include "config.hpp"
#include "disk_monitor.hpp"
#include "mqtt_facade.hpp"
#include "recording_manager.hpp"
#include "ros2_monitor.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>

using namespace woeden;
using namespace std;
using namespace placeholders;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  string host = getenv("HOST");
  string home = getenv("HOME");

  config c(home);

  auto facade = make_shared<mqtt_facade>(host, c.get_id(), c.get_password());

  auto dm = make_shared<disk_monitor>(facade);
  auto rm = make_shared<recording_manager>(dm, facade);
  auto r2m = make_shared<ros2_monitor>(facade);

  facade->set_record_callback(bind(&recording_manager::start, rm, _1, _2, _3));
  facade->set_stop_callback(bind(&recording_manager::stop, rm));
  facade->set_upload_callback(bind(&recording_manager::upload, rm, _1, _2, _3));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dm);
  executor.add_node(rm);
  executor.add_node(r2m);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
