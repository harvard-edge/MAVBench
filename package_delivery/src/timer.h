#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <ios>
#include <fstream>
#include <chrono>
#include <ratio>

// Log a timestamp to file
void log_time(const std::string &fname, long timestamp);
void log_time(const std::string &fname);

#define LOG_TIME(fname)	log_time("/tmp/" #fname ".log")
#define RESET_TIMER()	auto _timer_start = std::chrono::system_clock::now()
#define LOG_ELAPSED(fname) do { \
    long _timer_duration = std::chrono::duration_cast<std::chrono::nanoseconds> \
                        (std::chrono::system_clock::now() - _timer_start).count(); \
    log_time("/tmp/" #fname ".log", _timer_duration); \
  } while(false)

#endif
