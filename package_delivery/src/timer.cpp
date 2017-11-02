#include "timer.h"

void log_time(const std::string &fname, long timestamp) {
  std::ofstream log(fname, std::ios_base::app | std::ios_base::out);
  log << timestamp << std::endl;
  log.close();
}

void log_time(const std::string &fname) {
  std::cout << "Timer: " << fname << "\n";

  std::ofstream log(fname, std::ios_base::app | std::ios_base::out);
  auto now = std::chrono::system_clock::now();
  auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
  auto value = now_ms.time_since_epoch();
  long timestamp = value.count();
  log << timestamp << std::endl;
  log.close();
}
