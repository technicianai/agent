cmake_minimum_required(VERSION 3.5)
project(woeden_agent)

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
find_package(diagnostic_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(SQLite3 REQUIRED)

add_executable(woeden_agent
  src/main.cpp
  src/recording_manager.cpp
  src/disk_monitor.cpp
  src/recording_dto.cpp
  src/recording_trigger.cpp
  src/ros2_monitor.cpp
  src/mqtt_facade.cpp
  src/utils.cpp
  src/config.cpp
)
ament_target_dependencies(woeden_agent diagnostic_msgs rclcpp std_msgs)
target_link_libraries(woeden_agent paho-mqttpp3 nlohmann_json::nlohmann_json SQLite::SQLite3 uuid)
target_include_directories(woeden_agent PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS woeden_agent
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
