#include "development_tools/tools.h"

namespace dev_tools {

const std::string RESET = "\033[0m";
const std::string BLACK = "\033[30m";                /* Black */
const std::string RED = "\033[31m";                  /* Red */
const std::string GREEN = "\033[32m";                /* Green */
const std::string YELLOW = "\033[33m";               /* Yellow */
const std::string BLUE = "\033[34m";                 /* Blue */
const std::string MAGENTA = "\033[35m";              /* Magenta */
const std::string CYAN = "\033[36m";                 /* Cyan */
const std::string WHITE = "\033[37m";                /* White */
const std::string REDPURPLE = "\033[95m";            /* Red Purple */
const std::string BOLDBLACK = "\033[1m\033[30m";     /* Bold Black */
const std::string BOLDRED = "\033[1m\033[31m";       /* Bold Red */
const std::string BOLDGREEN = "\033[1m\033[32m";     /* Bold Green */
const std::string BOLDYELLOW = "\033[1m\033[33m";    /* Bold Yellow */
const std::string BOLDBLUE = "\033[1m\033[34m";      /* Bold Blue */
const std::string BOLDMAGENTA = "\033[1m\033[35m";   /* Bold Magenta */
const std::string BOLDCYAN = "\033[1m\033[36m";      /* Bold Cyan */
const std::string BOLDWHITE = "\033[1m\033[37m";     /* Bold White */
const std::string BOLDREDPURPLE = "\033[1m\033[95m"; /* Bold Red Purple */

/// print the run time
void Timer::PrintAll() {
  std::cout << ">>> ===== Printing run time =====" << std::endl;
  for (const auto& r : records_) {
    double time_temp = std::accumulate(r.second.time_usage_in_ms_.begin(),
                                       r.second.time_usage_in_ms_.end(),
                                       0.0) /
                       double(r.second.time_usage_in_ms_.size());
    auto max_iter = std::max_element(r.second.time_usage_in_ms_.begin(),
                                     r.second.time_usage_in_ms_.end());
    auto min_iter = std::min_element(r.second.time_usage_in_ms_.begin(),
                                     r.second.time_usage_in_ms_.end());
    // LOG(INFO) << "> [ " << r.first << " ] average time usage: " << time_temp
    //           << " ms , called times: " << r.second.time_usage_in_ms_.size();

    std::cout << "> [ " << r.first << " ] average time usage: " << time_temp
              << "ms, min: " << *min_iter << ", max: " << *max_iter
              << ", called times: " << r.second.time_usage_in_ms_.size()
              << std::endl;
  }
}

// for common
Logger::Logger(int argc, char** argv, Config config) {
  // config
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);

  FLAGS_colorlogtostderr = config.colorlogtostderr;
  FLAGS_logbufsecs = config.logbufsecs;
  FLAGS_max_log_size = config.max_log_size;
  FLAGS_stop_logging_if_full_disk = config.stop_logging_if_full_disk;
  FLAGS_alsologtostderr = config.alsologtostderr;
  FLAGS_log_prefix = config.log_prefix;

  std::string current_path = boost::filesystem::current_path().c_str();
  std::string log_path = current_path + "/log";
  if (!boost::filesystem::exists(log_path)) {
    boost::filesystem::create_directories(log_path);
  }

  std::cout << "log path: " << log_path << std::endl;
  FLAGS_log_dir = log_path;
}

// for ROS
Logger::Logger(int argc, char** argv, std::string current_path, Config config) {
  // config
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);

  FLAGS_colorlogtostderr = config.colorlogtostderr;
  FLAGS_logbufsecs = config.logbufsecs;
  FLAGS_max_log_size = config.max_log_size;
  FLAGS_stop_logging_if_full_disk = config.stop_logging_if_full_disk;
  FLAGS_alsologtostderr = config.alsologtostderr;
  FLAGS_log_prefix = config.log_prefix;

  std::string log_path = current_path + "/log";
  if (!boost::filesystem::exists(log_path)) {
    boost::filesystem::create_directories(log_path);
  }

  std::cout << "log path: " << log_path << std::endl;
  FLAGS_log_dir = log_path;
}

Logger::~Logger() {
  google::ShutdownGoogleLogging();
  std::cout << "Logger Is Finshed!" << std::endl;
}

}  // namespace dev_tools