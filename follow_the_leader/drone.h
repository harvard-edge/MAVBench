#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <list>
#include <opencv2/opencv.hpp>
#include "rpc/RpcLibClient.hpp"
#include "coord.h"
#include "configs.h"
// Control functions
class Drone {
public:
	Drone();
	Drone(const std::string& ip_addr, uint16_t port);

	~Drone();

	// Control functions
	void arm();
	void disarm();
	void connect();
	void connect(const std::string& ip_addr, uint16_t port);
	bool takeoff(double h);
	bool fly_velocity(double vx, double vy, double vz, double duration = 2);
	bool set_yaw(float y);
	float get_yaw();
	float get_roll();
	float get_pitch();
	bool land();

	// Camera functions
	cv::Mat read_frame();
	cv::Mat read_frame_depth();
	cv::Mat read_frame_buf(bool * empty_flag = 0);
	cv::Mat read_frame_buf_newest();
    std::vector<cv::Mat> buffered_images();
	void start_buffering();
	void stop_buffering();
	void clear_buffer();
	bool buffer_empty();

	// GPS functions
	coord gps();

private:
	void buffer_images();

	msr::airlib::RpcLibClient * client;
	std::list<cv::Mat> cam_buf;
	std::mutex cam_buf_mutex;
	std::thread cam_thread;
	std::atomic<bool> stop_buffer_flag;
};

#endif
