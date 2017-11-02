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
#define RESET_TIMER()	auto start = std::chrono::system_clock::now()
#define LOG_ELAPSED(fname) do { \
    long duration = std::chrono::duration_cast<std::chrono::nanoseconds> \
                        (std::chrono::system_clock::now() - start).count(); \
    log_time("/tmp/" #fname ".log", duration); \
  } while(false)

#endif
