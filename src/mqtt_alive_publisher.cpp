#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mqtt/client.h"
#include "mqtt/connect_options.h"

class mqtt_alive_publisher : public rclcpp::Node
{
public:
  mqtt_alive_publisher() : Node("mqtt_alive_publisher"), client_("tcp://localhost:1883", "paho-cpp-data-publish", nullptr)
  {
    auto connect_options = mqtt::connect_options_builder() 
        .keep_alive_interval(std::chrono::seconds(20))
        .clean_session()
        .finalize();
    client_.connect(connect_options);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(60), std::bind(&mqtt_alive_publisher::timer_callback, this));
  }

  ~mqtt_alive_publisher()
  {
    client_.disconnect();
  }

private:
  void timer_callback()
  {
    try {
        const char* payload = "alive";
        mqtt::message_ptr msg = mqtt::make_message("alive", payload);
        client_.publish(msg);
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(get_logger(), "mqtt error");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  mqtt::client client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mqtt_alive_publisher>());
  rclcpp::shutdown();
  return 0;
}
