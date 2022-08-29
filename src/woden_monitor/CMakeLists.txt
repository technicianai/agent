cmake_minimum_required(VERSION 3.5)
project(woeden_monitor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
  paho-mqtt-cpp
  GIT_REPOSITORY "https://github.com/eclipse/paho.mqtt.cpp.git"
  GIT_TAG "2ff3d155dcd10564f1816675789284b4efd79eb7"
)
FetchContent_Declare(
  json 
  URL https://github.com/nlohmann/json/releases/download/v3.11.1/json.tar.xz
)
FetchContent_MakeAvailable(paho-mqtt-cpp json)

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(SQLite3 REQUIRED)

add_executable(woeden_monitor src/woeden_monitor.cpp)
ament_target_dependencies(woeden_monitor rclcpp std_msgs)
#target_link_libraries(woeden_monitor paho-mqttpp3 paho-mqtt3as nlohmann_json::nlohmann_json SQLite::SQLite3 OpenSSL::SSL)
target_link_libraries(woeden_monitor paho-mqttpp3 nlohmann_json::nlohmann_json SQLite::SQLite3)

install(TARGETS woeden_monitor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
