
#ifndef COMMON_LOGGER_GLOG_HELPER_H_
#define COMMON_LOGGER_GLOG_HELPER_H_

#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <glog/raw_logging.h>
#include <unistd.h>

namespace common {

void SignalHandle(const char* data, int size) {
  std::string str = std::string(data, size);
  LOG(ERROR) << str;
}

class GlogHelper {
 public:
  explicit GlogHelper(const char* program) {
    int ret = 0;
    ret = access((std::string(FLAGS_log_dir)).c_str(), R_OK|W_OK);
    if (ret != 0) {
      std::string str_mkdir = "mkdir " + static_cast<std::string>(FLAGS_log_dir);
      system(str_mkdir.c_str());
    }
    FLAGS_logbufsecs = 0;
    FLAGS_stderrthreshold = google::WARNING;
    FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;
    google::InitGoogleLogging(program);
    google::InstallFailureSignalHandler();
  }
  ~GlogHelper() { google::ShutdownGoogleLogging(); }
};

} // namespace common


#endif // COMMON_LOGGER_GLOG_HELPER_H_