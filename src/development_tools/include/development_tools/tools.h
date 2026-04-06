#pragma once
#include <glog/logging.h>
#include <stdlib.h>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <vector>

namespace dev_tools {

extern const std::string RESET;
extern const std::string BLACK;
extern const std::string RED;
extern const std::string GREEN;
extern const std::string YELLOW;
extern const std::string BLUE;
extern const std::string MAGENTA;
extern const std::string CYAN;
extern const std::string WHITE;
extern const std::string REDPURPLE;
extern const std::string BOLDBLACK;
extern const std::string BOLDRED;
extern const std::string BOLDGREEN;
extern const std::string BOLDYELLOW;
extern const std::string BOLDBLUE;
extern const std::string BOLDMAGENTA;
extern const std::string BOLDCYAN;
extern const std::string BOLDWHITE;
extern const std::string BOLDREDPURPLE;

class Timer {
 public:
  struct TimerRecord {
    TimerRecord() = default;
    TimerRecord(const std::string& name, double time_usage) {
      func_name_ = name;
      time_usage_in_ms_.emplace_back(time_usage);
    }
    std::string func_name_;
    std::vector<double> time_usage_in_ms_;
  };

  template <class F>
  void Evaluate(F&& func, const std::string& func_name) {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::forward<F>(func)();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count() *
        1000;
    if (records_.find(func_name) != records_.end()) {
      records_[func_name].time_usage_in_ms_.emplace_back(time_used);
    } else {
      records_.insert({func_name, TimerRecord(func_name, time_used)});
    }
  }

  void PrintAll();

  /// clean the records
  void Clear() { records_.clear(); }

  std::map<std::string, TimerRecord> records_;
};

class Logger {
 public:
  struct Config {
    Config() {};

    bool colorlogtostderr = true;
    int logbufsecs = 0;
    int max_log_size = 100;
    bool stop_logging_if_full_disk = true;
    bool alsologtostderr = true;
    bool log_prefix = true;
  };

  Logger(int argc, char** argv, Config config = Config());

  Logger(int argc,
         char** argv,
         std::string current_path,
         Config config = Config());

  ~Logger();
};
}  // namespace dev_tools