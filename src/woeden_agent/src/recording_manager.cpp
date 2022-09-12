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

namespace woeden
{
static int sql_callback(void *NotUsed, int argc, char **argv, char **azColName) {
   return 0;
}

recording_manager::recording_manager(shared_ptr<disk_monitor> dm, shared_ptr<mqtt_facade> facade) : Node("woeden_recording_manager"), facade_(facade)
{
  dm_ = dm;
  recording_ = false;
}

void recording_manager::start(uint32_t bag_id, string base_path, uint32_t duration, vector<recording_topic> recording_topics)
{
  if (recording_) {
    throw logic_error("already recording");
  }
  bag_id_ = bag_id;
  base_path_ = base_path;
  bag_path_ = base_path_ + "/woeden/bags/" + to_string(bag_id_);

  for (const recording_topic & rt : recording_topics) {
    if (rt.throttle) {
      pid_t pid = fork();
      if (pid == 0) {
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
    record_cmd(bag_path_, recording_topics);
  } else if (recording_pid_ > 0) {
    recording_ = true;
    facade_->publish_started(bag_id_);
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

void recording_manager::stop()
{
  if (recording_pid_ <= 0) {
    throw logic_error("only the parent can stop recording");
  }

  kill(recording_pid_, SIGINT);
  for (int pid : throttle_pids_) {
    int pgid = getpgid(pid);
    killpg(pgid, SIGKILL);
  }
  sleep(10);

  for (const auto & entry : std::filesystem::directory_iterator(bag_path_)) {
    std::string db_path = entry.path();
    if (db_path.find(".db3") != std::string::npos) {
      remote_throttle_from_db(db_path.c_str());
    }
  }

  string metadata_path = bag_path_ + "/metadata.yaml";
  string metadata = load_metadata(metadata_path);
  metadata = remote_throttle_from_metadata(metadata);
  update_metadata(metadata_path, metadata);

  facade_->publish_stopped(bag_id_, {
    .metadata = metadata,
    .size = bag_size()
  });

  timer_.reset();
  recording_ = false;
}

bool recording_manager::is_recording()
{
  return recording_;
}

void recording_manager::throttle_cmd(string topic, double frequency)
{
  setsid();
  char* args[8];
  int i = 0;

  args[i++] = "ros2";
  args[i++] = "run";
  args[i++] = "topic_tools";
  args[i++] = "throttle";
  args[i++] = "messages";

  const char* in_topic = topic.c_str();
  args[i] = (char*) malloc(topic.length()+1);
  strcpy(args[i++], in_topic);

  const char* msgs_per_sec = to_string(frequency).c_str();
  args[i] = (char*) malloc(strlen(msgs_per_sec)+1);
  strcpy(args[i++], msgs_per_sec);

  std::string out_topic_str = topic + "/throttle";
  const char* out_topic = out_topic_str.c_str();
  args[i] = (char*) malloc(strlen(out_topic)+1);
  strcpy(args[i++], out_topic);

  args[i] = NULL;

  execvp("ros2", args);
}

void recording_manager::record_cmd(string bag_path, vector<recording_topic> recording_topics)
{
  char* args[recording_topics.size() + 6] = { "ros2", "bag", "record" };
  int i = 3;

  for (const recording_topic & rt : recording_topics) {
    string topic_str = rt.name;
    if (rt.throttle) {
      topic_str += "/throttle";
    }
    const char* name = topic_str.c_str();
    args[i] = (char*) malloc(strlen(name)+1);
    strcpy(args[i++], name);
  }

  args[i++] = "-o";
  args[i++] = const_cast<char*>(bag_path.c_str());
  args[i++] = NULL;

  execvp("ros2", args);
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
  return directory_size(bag_path_);
}

uintmax_t recording_manager::directory_size(string path)
{
  uintmax_t size = 0;
  for (const filesystem::directory_entry& f : filesystem::recursive_directory_iterator(path)) {
    if (filesystem::is_regular_file(f.path())) {
      size += filesystem::file_size(f.path());
    }
  }
  return size;
}

string recording_manager::load_metadata(string metadata_path)
{
  ifstream t(metadata_path);
  stringstream buffer;
  buffer << t.rdbuf();
  return buffer.str();
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
  size_ = bag_size();

  double rate = (size_ - previous_size_) / SAMPLING_INTERVAL;
  double eta = rate > 0 ? (double) remaining / rate : 0;

  facade_->publish_status(bag_id_, {
    .eta = eta,
    .rate = rate,
    .size = size_
  });
}

void recording_manager::upload(uint32_t bag_id, string base_path, vector<string> urls)
{
  string command = "python3 /woeden_agent/bag_utils/upload.py ";
  command += to_string(bag_id) + " ";
  command += base_path + " ";
  for (const string & url : urls) {
    command += "\"" + url + "\" ";
  }

  auto f = [&, command]() {
    string result = blocking_cmd(command.c_str());
    facade_->publish_uploaded(result);
  };

  thread thread_object(f);
  thread_object.detach();
}
}
