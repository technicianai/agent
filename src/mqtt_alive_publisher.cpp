#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>

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

    std::ifstream id_file;
    id_file.open("/tmp/woden/id");
    int id;
    id_file >> id;
    std::stringstream stream;
    stream << "/";
    stream << id;
    stream << "/alive";
    
    std::function<void ()> callback = std::bind(&mqtt_alive_publisher::timer_callback, this, stream.str());
    timer_ = this->create_wall_timer(std::chrono::seconds(1), callback);
  }

  ~mqtt_alive_publisher()
  {
    client_.disconnect();
  }

private:
  void timer_callback(std::string topic)
  {
    try {
        const char* payload = "1";
        RCLCPP_INFO(get_logger(), topic.c_str());
        mqtt::message_ptr msg = mqtt::make_message(topic, payload);
        client_.publish(msg);
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(get_logger(), exc.what());
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
