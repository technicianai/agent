#include "recording_manager.hpp"
#include "utils.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>

#include <signal.h>
#include <sqlite3.h> 
#include <string.h>
#include <stdio.h>

using namespace std;
using namespace placeholders;

namespace woeden
{
static int sql_callback(void *NotUsed, int argc, char **argv, char **azColName) {
   return 0;
}

recording_manager::recording_manager(shared_ptr<disk_monitor> dm, shared_ptr<mqtt_facade> facade, always_record_config arc, double max_bandwidth) : Node("woeden_recording_manager"), facade_(facade)
{
  dm_ = dm;
  recording_ = false;
  stopping_ = false;
  trigger_id_ = NULL;
  always_record_config_ = arc;
  max_bandwidth_ = max_bandwidth;
  if (arc.enabled) {
    start_always_record();
  }
  upload_client_ = this->create_client<interfaces::srv::Upload>("/upload_bag");
  upload_subscription_ = this->create_subscription<interfaces::msg::UploadBytes>("upload_chunks", 10, bind(&recording_manager::upload_bytes, this, _1));
  upload_complete_ = this->create_service<interfaces::srv::UploadComplete>("/upload_complete", bind(&recording_manager::upload_complete, this, _1, _2));
}

void recording_manager::start(string bag_uuid, string base_path, uint32_t duration, bool temp, vector<recording_topic> recording_topics)
{
  if (recording_) {
    return;
  }
  bag_uuid_ = bag_uuid;
  base_path_ = base_path;
  bag_path_ = base_path_ + "/woeden/bags/" + bag_uuid_;
  if (temp) {
    bag_path_ += "_temp.bag";
  } else {
    bag_path_ += ".bag";
  }
  
  for (const recording_topic & rt : recording_topics) {
    if (rt.throttle) {
      pid_t pid = fork();
      if (pid == 0) {
        close(2);
        throttle_cmd(rt.name, rt.frequency);
      } else if (pid > 0) {
        throttle_pids_.push_back(pid);
      } else {
        throw runtime_error("error forking throttle process");
      }
    }
  }

  recording_pid_ = fork();
  if (recording_pid_ == 0) {
    close(2);
    record_cmd(bag_path_, recording_topics);
  } else if (recording_pid_ > 0) {
    recording_ = true;
    facade_->publish_started(bag_uuid_, trigger_id_);
    if (!temp) {
      function<void ()> status_check = bind(&recording_manager::status_check, this);
      timer_ = create_wall_timer(chrono::seconds(15), status_check);
      if (duration > 0) {
        auto_stop_timer_ = create_wall_timer(chrono::seconds(duration), [&]() -> void {
          stop(false);
          auto_stop_timer_.reset();
        });
      }
    }
  } else {
    throw runtime_error("error forking recording process");
  }
}

void recording_manager::auto_start(recording_trigger rt)
{
  trigger_id_ = rt.get_id();
  if (!always_record_config_.enabled) {
    start(uuid(), rt.get_base_path(), rt.get_duration(), false, rt.get_record_topics());
    return;
  }

  if (stopping_) {
    return;
  }

  if (!recording_) {
    trigger_start_time_ = time(0);
    trigger_duration_ = rt.get_duration();

    string bag_uuid = uuid();
    start(bag_uuid, rt.get_base_path(), rt.get_duration(), true, rt.get_record_topics());

    string buffer_path = rt.get_base_path() + "/woeden/bags/" + bag_uuid + "_buffer.bag";
    if (fork() == 0) buffer_dump_cmd(buffer_path);
    if (fork() == 0) buffer_pause_cmd();
  } else {
    auto_stop_timer_.reset();
  }

  auto_stop_timer_ = create_wall_timer(chrono::seconds(trigger_duration_), [&]() -> void {
    auto_stop_timer_.reset();
    stop(true);
    if (fork() == 0) buffer_resume_cmd();
  });
}

void recording_manager::stop(bool merge_buffer)
{
  if (recording_pid_ <= 0) {
    throw logic_error("only the parent can stop recording");
  }

  timer_.reset();
  stopping_ = true;

  kill(recording_pid_, SIGINT);
  for (int pid : throttle_pids_) {
    int pgid = getpgid(pid);
    killpg(pgid, SIGKILL);
  }

  sleep(10);

  if (merge_buffer) {
    string path = base_path_ + "/woeden/bags/";
    string buffer_path = path + bag_uuid_ + "_buffer.bag";
    string temp_path = path + bag_uuid_ + "_temp.bag";
    string cmd = "rosbag-merge --input_bags " + buffer_path + " " + temp_path
                  + " --output_path " + path + " --outbag_name " + bag_uuid_;
    bag_path_ = path + bag_uuid_ + ".bag";
    blocking_cmd(cmd.c_str());
    remove(temp_path.c_str());
    remove(buffer_path.c_str());
  }

  uintmax_t size = bag_size();
  facade_->publish_stopped(bag_uuid_, size);

  recording_ = false;
  stopping_ = false;
  trigger_id_ = NULL;
  recording_pid_ = NULL;
}

bool recording_manager::is_recording()
{
  return recording_;
}

void recording_manager::throttle_cmd(string topic, double frequency)
{
  setsid();
  char* args[7];
  int i = 0;

  args[i++] = "rosrun";
  args[i++] = "topic_tools";
  args[i++] = "throttle";
  args[i++] = "messages";

  const char* in_topic = topic.c_str();
  args[i] = (char*) malloc(topic.length()+1);
  strcpy(args[i++], in_topic);

  const char* msgs_per_sec = to_string(frequency).c_str();
  args[i] = (char*) malloc(strlen(msgs_per_sec)+1);
  strcpy(args[i++], msgs_per_sec);

  string out_topic_str = topic + "/throttle";
  const char* out_topic = out_topic_str.c_str();
  args[i] = (char*) malloc(strlen(out_topic)+1);
  strcpy(args[i++], out_topic);

  args[i] = NULL;

  execvp("rosrun", args);
}

void recording_manager::always_record_cmd(string bag_path)
{
  char* args[6] = {
    "rosbag", "record",
    "-a",
    "-O", const_cast<char*>(bag_path.c_str()),
    NULL
  };
  execvp("rosbag", args);
}

void recording_manager::buffer_run_cmd(uint32_t duration)
{
  string duration_string = to_string(duration);
  char* args[7] ={
    "rosrun", "rosbag_snapshot", "snapshot",
    "-a",
    "-d", const_cast<char*>(duration_string.c_str()),
    NULL
  };
  execvp("rosrun", args);
}

void recording_manager::buffer_dump_cmd(string buffer_path)
{
  char* args[7] = {
    "rosrun", "rosbag_snapshot", "snapshot",
    "-t",
    "-O", const_cast<char*>(buffer_path.c_str()),
    NULL
  };
  execvp("rosrun", args);
}

void recording_manager::buffer_pause_cmd()
{
  char* args[5] = { "rosrun", "rosbag_snapshot", "snapshot", "-p", NULL };
  execvp("rosrun", args);
}

void recording_manager::buffer_resume_cmd()
{
  char* args[5] = { "rosrun", "rosbag_snapshot", "snapshot", "-r", NULL };
  execvp("rosrun", args);
};

void recording_manager::record_cmd(string bag_path, vector<recording_topic> recording_topics)
{
  char* args[recording_topics.size() + 6] = { "rosbag", "record", "__name:=trigger_bag" };
  int i = 2;

  for (const recording_topic & rt : recording_topics) {
    string topic_str = rt.name;
    if (rt.throttle) {
      topic_str += "/throttle";
    }
    const char* name = topic_str.c_str();
    args[i] = (char*) malloc(strlen(name)+1);
    strcpy(args[i++], name);
  }

  args[i++] = "-O";
  args[i++] = const_cast<char*>(bag_path.c_str());

  // args[i++] = "--max-bag-size";
  // args[i++] = "1050000000"; // ~0.98GB

  args[i++] = NULL;

  execvp("rosbag", args);
}

void recording_manager::remote_throttle_from_db(const char* db_path)
{
  sqlite3 *db;
  int rc = sqlite3_open(db_path, &db);
  char *zErrMsg = 0;
  rc = sqlite3_exec(db, "UPDATE topics SET name=replace(name, '/throttle', '');", sql_callback, 0, &zErrMsg);
  sqlite3_close(db);
}

uintmax_t recording_manager::bag_size()
{
  return bag_size(bag_path_);
}

uintmax_t recording_manager::bag_size(string bag_path)
{
  return filesystem::file_size(bag_path);
}

string recording_manager::load_metadata(string path)
{
  string cmd = "rosbag info -y " + path;
  string output = blocking_cmd(cmd.c_str());
  return output;
}

string recording_manager::remote_throttle_from_metadata(string metadata)
{
  string text = "/throttle";
  size_t pos = 0;
  while ((pos = metadata.find(text, pos)) != string::npos) {
    metadata.replace(pos, text.length(), "");
  }
  return metadata;
}

void recording_manager::update_metadata(string metadata_path, string metadata)
{
  ofstream out(metadata_path);
  out << metadata;
  out.close();
}

void recording_manager::status_check()
{
  long remaining = dm_->remaining(base_path_);
  if (remaining <= 0) {
    stop(false);
  }

  try {
    previous_size_ = size_;
    string active_path = bag_path_ + ".active";
    size_ = bag_size(active_path);

    double rate = (size_ - previous_size_) / SAMPLING_INTERVAL;
    double eta = rate > 0 ? (double) remaining / rate : 0;

    facade_->publish_status(bag_uuid_, {
      .eta = eta,
      .rate = rate,
      .size = size_,
      .trigger_id = trigger_id_
    });
  } catch (std::filesystem::filesystem_error & e) {}
}

void recording_manager::upload(string bag_uuid, string base_path)
{
  auto request = std::make_shared<interfaces::srv::Upload::Request>();
  request->bag_uuid = bag_uuid;
  request->base_path = base_path;
  request->max_bandwidth = max_bandwidth_;

  while (!upload_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
  }

  auto result = upload_client_->async_send_request(request);
  result.wait();
}

void recording_manager::metadata_on_reconnect()
{
  for (mount& mnt : dm_->get_mounts()) {
    string base_path = mnt.path + "/woeden/bags/";
    if (filesystem::exists(base_path)) {
      for (const auto& entry : filesystem::directory_iterator(base_path)) {
        string path = entry.path().c_str();
        if (!filesystem::is_directory(path)) {
          string metadata = load_metadata(path);
          string bag_uuid = path.substr(path.find_last_of("/\\") + 1);
          uintmax_t size = filesystem::file_size(path);
          facade_->publish_stopped(bag_uuid, size);
        }
      }
    }
  }
}

void recording_manager::gif_upload(string bag_uuid, string base_path, string urls)
{
  // DO NOTHING

  // string command = "python3 /woeden_agent/bag_utils/gif.py ";
  // command += bag_uuid + " " + base_path + " '" + urls + "'";

  // auto f = [&, command, bag_uuid]() {
  //   blocking_cmd(command.c_str());
  //   facade_->publish_gif_uploaded(bag_uuid);
  // };

  // thread thread_object(f);
  // thread_object.detach();
}

void recording_manager::metadata_upload(string bag_uuid, string base_path)
{
  string bag_path = base_path + "/woeden/bags/" + bag_uuid + ".bag";
  string metadata = load_metadata(bag_path);
  facade_->publish_metadata(bag_uuid_, metadata);
}

void recording_manager::set_always_record(always_record_config arc)
{
  if (arc.enabled && !always_record_config_.enabled) {
    start_always_record();
  } else if (!arc.enabled && always_record_config_.enabled) {
    stop_always_record();
  }
  always_record_config_ = arc;
}

void recording_manager::start_always_record()
{
  base_path_ = always_record_config_.base_path;
  
  buffer_pid_ = fork();
  if (buffer_pid_ == 0) {
    close(2);
    buffer_run_cmd(always_record_config_.duration);
  } else if (buffer_pid_ < 0) {
    throw runtime_error("error forking recording process");
  }
}

void recording_manager::stop_always_record()
{
  timer_.reset();
  auto_stop_timer_.reset();

  kill(buffer_pid_, SIGKILL);

  recording_ = false;
  stopping_ = false;
  trigger_id_ = NULL;
  recording_pid_ = NULL;
}

void recording_manager::set_max_bandwidth(double bw)
{
  max_bandwidth_ = bw;
}

void recording_manager::upload_bytes(shared_ptr<interfaces::msg::UploadBytes> msg)
{
  const void * data = msg->contents.data();
  int length = msg->contents.size();
  facade_->publish_chunk(msg->bag_uuid, msg->index, data, length);
}

void recording_manager::upload_complete(shared_ptr<interfaces::srv::UploadComplete::Request> request, shared_ptr<interfaces::srv::UploadComplete::Response> response)
{
  facade_->publish_upload_complete(request->bag_uuid, request->chunks);
  response->success = true;
}
}
