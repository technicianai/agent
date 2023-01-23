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
  always_record_turn_ = true;
  always_record_config_ = arc;
  max_bandwidth_ = max_bandwidth;
  if (arc.enabled) {
    start_always_record();
  }
  upload_client_ = this->create_client<interfaces::srv::Upload>("/upload_bag");
  upload_subscription_ = this->create_subscription<interfaces::msg::UploadBytes>("upload_chunks", 10, bind(&recording_manager::upload_bytes, this, _1));
  upload_complete_ = this->create_service<interfaces::srv::UploadComplete>("/upload_complete", bind(&recording_manager::upload_complete, this, _1, _2));
}

void recording_manager::start(string bag_uuid, string base_path, uint32_t duration, vector<recording_topic> recording_topics)
{
  if (recording_ || always_record_config_.enabled) {
    return;
  }
  bag_uuid_ = bag_uuid;
  base_path_ = base_path;
  bag_path_ = base_path_ + "/woeden/bags/" + bag_uuid_ + ".bag";

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
    function<void ()> status_check = bind(&recording_manager::status_check, this);
    timer_ = create_wall_timer(chrono::seconds(15), status_check);
    if (duration > 0) {
      auto_stop_timer_ = create_wall_timer(chrono::seconds(duration), [&]() -> void {
        stop();
        auto_stop_timer_.reset();
      });
    }
  } else {
    throw runtime_error("error forking recording process");
  }
}

void recording_manager::auto_start(recording_trigger rt)
{
  trigger_id_ = rt.get_id();
  if (!always_record_config_.enabled) {
    start(uuid(), rt.get_base_path(), rt.get_duration(), rt.get_record_topics());
    return;
  }

  if (stopping_ || always_record_pid_1_ == NULL || always_record_pid_2_ == NULL) {
    return;
  }

  always_record_timer_.reset();

  if (!recording_) {
    recording_ = true;

    always_record_timer_.reset();
    trigger_start_time_ = time(0);
    trigger_duration_ = rt.get_duration();
    
    if ((always_record_turn_ && always_record_pid_1_ != NULL) || (!always_record_turn_ && always_record_pid_2_ == NULL && always_record_pid_1_ != NULL)) {
      recording_pid_ = always_record_pid_1_;
      bag_uuid_ = always_record_bag_uuid_1_;
      bag_path_ = always_record_bag_path_1_;
      annihilate_recording(always_record_pid_2_, always_record_bag_path_2_);
    } else if ((!always_record_turn_ && always_record_pid_2_ != NULL) || (always_record_turn_ && always_record_pid_1_ == NULL && always_record_pid_2_ != NULL)) {
      recording_pid_ = always_record_pid_2_;
      bag_uuid_ = always_record_bag_uuid_2_;
      bag_path_ = always_record_bag_path_2_;
      annihilate_recording(always_record_pid_1_, always_record_bag_path_1_);
    }

    function<void ()> status_check = bind(&recording_manager::status_check, this);
    timer_ = create_wall_timer(chrono::seconds(15), status_check);
  } else {
    time_t current_time = time(0);
    time_t time_left = trigger_start_time_ + trigger_duration_ - current_time;
    if (time_left < rt.get_duration()) {
      trigger_duration_ += rt.get_duration() - time_left;
      auto_stop_timer_.reset();
    }
  }

  auto_stop_timer_ = create_wall_timer(chrono::seconds(trigger_duration_), [&]() -> void {
    auto_stop_timer_.reset();
    stop();
    always_record_pid_1_ = NULL;
    always_record_pid_2_ = NULL;
    start_always_record();
  });
}

void recording_manager::stop()
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

  // char* args[4] = { "rosnode", "kill", "/trigger_bag", NULL };
  // if (fork() > 0) {
  //   execvp("rosnode", args);
  // }
  sleep(10);

  // for (const auto & entry : filesystem::directory_iterator(bag_path_)) {
  //   string db_path = entry.path().string();
  //   if (db_path.find(".db3") != string::npos) {
  //     remote_throttle_from_db(db_path.c_str());
  //   }
  // }

  // string metadata_path = bag_path_ + "/metadata.yaml";
  // string metadata = load_metadata(metadata_path);
  // metadata = remote_throttle_from_metadata(metadata);
  // if (trigger_id_ != NULL) {
  //   metadata += "\ntrigger: " + to_string(trigger_id_);
  // }
  // update_metadata(metadata_path, metadata);
  string metadata = load_metadata(bag_path_);
  facade_->publish_stopped(bag_uuid_, {
    .metadata = metadata,
    .size = bag_size()
  });

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

void recording_manager::record_cmd(string bag_path, vector<recording_topic> recording_topics)
{
  char* args[recording_topics.size() + 6] = { "rosbag", "record", "__name:=trigger_bag"};
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
    stop();
  }

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
          facade_->publish_stopped(bag_uuid, {
            .metadata = metadata,
            .size = filesystem::file_size(path)
          });
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
  function<void ()> always_record = bind(&recording_manager::always_record, this);
  always_record();
  always_record_timer_ = create_wall_timer(chrono::seconds(always_record_config_.duration * 2), always_record);
  base_path_ = always_record_config_.base_path;
}

void recording_manager::stop_always_record()
{
  timer_.reset();
  always_record_timer_.reset();
  auto_stop_timer_.reset();
  if (always_record_pid_1_ != NULL) {
    annihilate_recording(always_record_pid_1_, always_record_bag_path_1_);
  }
  if (always_record_pid_2_ != NULL) {
    annihilate_recording(always_record_pid_2_, always_record_bag_path_2_);
  }
  recording_ = false;
  stopping_ = false;
  trigger_id_ = NULL;
  recording_pid_ = NULL;
}

void recording_manager::always_record()
{
  string bag_uuid = uuid();
  string always_record_bag_path = always_record_config_.base_path + "/woeden/bags/" + bag_uuid + ".bag";
  pid_t always_record_pid = fork();

  if (always_record_pid == 0) {
    close(2);
    always_record_cmd(always_record_bag_path);
  } else if (always_record_pid < 0) {
    throw runtime_error("error forking recording process");
  }

  if (always_record_turn_)
  {
    if (always_record_pid_1_ != NULL) {
      annihilate_recording(always_record_pid_1_, always_record_bag_path_1_);
    }
    always_record_pid_1_ = always_record_pid;
    always_record_bag_uuid_1_ = bag_uuid;
    always_record_bag_path_1_ = always_record_bag_path;
  } else {
    if (always_record_pid_2_ != NULL) {
      annihilate_recording(always_record_pid_2_, always_record_bag_path_2_);
    }
    always_record_pid_2_ = always_record_pid;
    always_record_bag_uuid_2_ = bag_uuid;
    always_record_bag_path_2_ = always_record_bag_path;
  }

  always_record_turn_ = !always_record_turn_;
}

void recording_manager::annihilate_recording(pid_t pid, string bag_path)
{
  kill(pid, SIGKILL);
  filesystem::remove_all(bag_path);
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
