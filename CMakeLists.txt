cmake_minimum_required(VERSION 3.5)
project(woden-monitor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(alive src/mqtt_alive_publisher.cpp)
ament_target_dependencies(alive rclcpp std_msgs)
target_link_libraries(alive paho-mqttpp3 paho-mqtt3a)

install(TARGETS alive
  DESTINATION lib/${PROJECT_NAME})

ament_package()
