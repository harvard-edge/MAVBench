#include "drone.h"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include "controllers/DroneControllerBase.hpp"
#include "controllers/VehicleCameraBase.hpp"
#include "common/Common.hpp"
#include "log__class.h"
#include <fstream>
using namespace std;
extern log__class log__f;
Drone::Drone() : client(0)
{
	connect();
}

Drone::Drone(const std::string& ip_addr, uint16_t port) : client(0)
{
	connect(ip_addr, port);
}

Drone::~Drone()
{
	if (client != 0)
		delete client;
}

void Drone::connect()
{
	if (client != 0)
		delete client;
	client = new msr::airlib::RpcLibClient();
}

void Drone::connect(const std::string& ip_addr, uint16_t port)
{
	if (client != 0)
		delete client;
	client = new msr::airlib::RpcLibClient(ip_addr, port);
}

void Drone::arm()
{
	client->armDisarm(true);
}

void Drone::disarm()
{
	client->armDisarm(false);
}

bool Drone::takeoff(double h)
{
	const double takeoff_timeout = 60.0;
    auto ground_pos = client->getPosition();

	try {
		client->takeoff(takeoff_timeout);
	} catch (...) {
		std::cout << "Taking off failed" << std::endl;
	}

	std::cout << "Press enter to set offboard mode";
	while (std::cin.get() != '\n') {}

	try {
		client->setOffboardMode(true);
	} catch (...) {
		std::cout << "Setting offboard mode failed" << std::endl;
	}

	std::cout << "Press enter to move to height " << h;
	while (std::cin.get() != '\n') {}

	try{
    	client->moveToPosition(ground_pos.x(), ground_pos.y(), ground_pos.z() - h, 1);
	} catch (...) {
		std::cout << "Moving to z position failed" << std::endl;
	}

	return true;
}

bool Drone::set_yaw(float y)
{
	try {
		client->rotateToYaw(y, 60, 5);
	}catch(...){
		std::cerr << "set_yaw failed" << std::endl;
		return false;
	}

	return true;
}

bool Drone::fly_velocity(double vx, double vy, double vz, double duration)
{
	try {
		client->moveByVelocity(vx, vy, vz, duration);
	} catch(...) {
		std::cerr << "fly_velocity failed" << std::endl;
		return false;
	}

	return true;
}

bool Drone::land()
{
	try {
		client->land();
	} catch(...) {
		std::cerr << "land failed" << std::endl;
		return false;
	}

	return true;
}

cv::Mat Drone::read_frame()
{
	cv::Mat result;
	cv::Mat scaled;
	const int max_tries = 1000000;
	std::vector<uint8_t> png;

	try {
        for (int i = 0; png.size() <= 1 && i < max_tries; i++) {
            png = client->simGetImage(0, msr::airlib::VehicleCameraBase::ImageType_::Scene);
		}
	} catch (...) {
		std::cerr << "read_frame failed" << std::endl;
	}

	if (png.size() > 1) {
#if CV_MAJOR_VERSION==3
		result = cv::imdecode(png, cv::IMREAD_COLOR);
#else
		result = cv::imdecode(png, CV_LOAD_IMAGE_COLOR);
#endif
	} else {
		std::cerr << "read_frame: Empty image returned" << std::endl;
	}

	return result;
}

cv::Mat Drone::read_frame_depth()
{
	cv::Mat result;
	cv::Mat scaled;
	const int max_tries = 1000000;
	std::vector<uint8_t> png;

	try {
		for (int i = 0; png.size() <= 1 && i < max_tries; i++) {
			png = client->simGetImage(0, msr::airlib::VehicleCameraBase::ImageType_::Depth);
		}
	} catch (...) {
		std::cerr << "read_frame_depth failed" << std::endl;
	}

	if (png.size() > 1) {
#if CV_MAJOR_VERSION==3
		result = cv::imdecode(png, cv::IMREAD_GRAYSCALE);
#else
		result = cv::imdecode(png, CV_LOAD_IMAGE_GRAYSCALE);
#endif
	} else {
		std::cerr << "read_frame: Empty image returned" << std::endl;
	}

	return result;
}

coord Drone::gps()
{
	auto pos = client->getPosition();
	return {pos.x(), pos.y(), pos.z()};
}

void Drone::buffer_images()
{
	while (!stop_buffer_flag) {
        cv::Mat img = read_frame();

	    cam_buf_mutex.lock();
		#ifdef DEBUG	
        log__f.write("mutex locked inside buffer_images");
        #endif
        cam_buf.push_back(img);
		cam_buf_mutex.unlock();
		#ifdef DEBUG	
        log__f.write("mutex UNlocked inside buffer_images");
        #endif

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void Drone::start_buffering()
{
	stop_buffer_flag = false;
	cam_thread = std::thread(&Drone::buffer_images, this);
}

void Drone::stop_buffering()
{
	stop_buffer_flag = true;

	try {
		cam_thread.join();
	} catch (...) { 
        log__f.write("exception in stop buffering");
        //exit(0);
        //TODO: add the correct exception here
	}
}

cv::Mat Drone::read_frame_buf(bool * empty_flag)
{
	cv::Mat result;

	cam_buf_mutex.lock();
	if (!cam_buf.empty()) {
		result = cam_buf.front();
		cam_buf.pop_front();
	}

	if (empty_flag)
		*empty_flag = cam_buf.empty();

	cam_buf_mutex.unlock();

	return result;
}

cv::Mat Drone::read_frame_buf_newest()
{
	cv::Mat result;

	cam_buf_mutex.lock();
	if (!cam_buf.empty()) {
		result = cam_buf.back();
	}
	cam_buf_mutex.unlock();

	return result;
}
    

std::vector<cv::Mat> Drone::buffered_images()
{
	cam_buf_mutex.lock();
    std::vector<cv::Mat> result(cam_buf.begin(), cam_buf.end());
	cam_buf_mutex.unlock();
    return result;
}


void Drone::clear_buffer()
{
    cam_buf_mutex.lock();
    #ifdef DEBUG	
    log__f.write("mutex locked inside clear_images");
    #endif
    cam_buf.clear();
	cam_buf_mutex.unlock();
    #ifdef DEBUG	
    log__f.write("mutex UNlocked inside clear_images");
    #endif
}

bool Drone::buffer_empty()
{
	bool result;
	cam_buf_mutex.lock();
	result = cam_buf.empty();
	cam_buf_mutex.unlock();
	return result;
}

float Drone::get_yaw()
{
	auto q = client->getOrientation();
	float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return y*180 / M_PI;
}

float Drone::get_roll()
{
	auto q = client->getOrientation();
	float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return r*180 / M_PI;
}

float Drone::get_pitch()
{
	auto q = client->getOrientation();
	float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return p*180 / M_PI;
}
