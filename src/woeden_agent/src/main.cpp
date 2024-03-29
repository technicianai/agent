#include <string>

#include "config.hpp"
#include "disk_monitor.hpp"
#include "mqtt_facade.hpp"
#include "rclcpp/rclcpp.hpp"
#include "recording_manager.hpp"
#include "ros2_monitor.hpp"

using namespace woeden;
using namespace std;
using namespace placeholders;

std::string get_env_or_default(string variable_name, string default_value)
{
  const char* value = getenv(variable_name.c_str());
  return value ? value : default_value;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  string host = get_env_or_default("MQTT_HOST", "ssl://mqtt.woeden.com");
  string home = get_env_or_default("HOME", "~/");

  config c(home);

  auto ca_cert = home + "/woeden/isrgrootx1.pem";
  auto facade = make_shared<mqtt_facade>(host, ca_cert, c.get_id(), c.get_password());

  auto dm = make_shared<disk_monitor>(facade);
  auto rm =
    make_shared<recording_manager>(dm, facade, c.get_always_record(), c.get_max_bandwidth());
  auto r2m = make_shared<ros2_monitor>(facade, rm, c.get_recording_triggers());

  facade->set_record_callback(bind(&recording_manager::start, rm, _1, _2, _3, _4));
  facade->set_stop_callback(bind(&recording_manager::stop, rm));
  facade->set_upload_callback(bind(&recording_manager::upload, rm, _1, _2));
  facade->set_new_trigger_callback([&c, r2m](recording_trigger rt) -> void {
    r2m->add_trigger(rt);
    c.add_trigger(rt);
  });
  facade->set_update_trigger_callback([&c, r2m](uint32_t id, bool enabled) -> void {
    r2m->update_trigger(id, enabled);
    c.update_trigger(id, enabled);
  });
  facade->set_update_always_record_callback(
    [&c, rm](uint32_t duration, bool enabled, string base_path) -> void {
      c.update_always_record(duration, enabled, base_path);
      rm->set_always_record(c.get_always_record());
    });
  facade->set_gateway_callback(bind(&ros2_monitor::open_gateway, r2m, _1));
  facade->set_gateway_close_callback(bind(&ros2_monitor::close_gateway, r2m));
  facade->set_reconnect_callback(bind(&recording_manager::metadata_on_reconnect, rm));
  facade->set_reconnect_callback([&c, rm, facade]() -> void {
    rm->metadata_on_reconnect();
    facade->publish_trigger_status(c.get_recording_triggers());
    facade->publish_always_record_status(c.get_always_record());
    facade->publish_max_bandwidth_status(c.get_max_bandwidth());
  });
  facade->set_gif_upload_callback(bind(&recording_manager::gif_upload, rm, _1, _2, _3));
  facade->set_update_max_bandwidth_callback([&c, rm](double max_bw) -> void {
    rm->set_max_bandwidth(max_bw);
    c.update_max_bandwidth(max_bw);
  });

  facade->connect();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dm);
  executor.add_node(rm);
  executor.add_node(r2m);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
