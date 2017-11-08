#include "Drone.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <chrono>
#include "common/Common.hpp"
#include "coord.h"
#include <geometry_msgs/Point.h>

Drone::Drone() : client(0)
{
	connect();
    initial_gps = gps();
}

Drone::Drone(const std::string& ip_addr, uint16_t port) : client(0)
{
	connect(ip_addr, port);
    initial_gps = gps();
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
    client->enableApiControl(true);
}

void Drone::connect(const std::string& ip_addr, uint16_t port)
{
	if (client != 0)
		delete client;
	client = new msr::airlib::RpcLibClient(ip_addr, port);
    client->enableApiControl(true);
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
		// client->setOffboardMode(true);
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
    float yaw_rate = 10.0;
    float time_to_spin = (y-get_yaw()) / yaw_rate;

    if (time_to_spin < 0) {
        yaw_rate *= -1;
        time_to_spin *= -1;
    }

	try {
		client->rotateToYaw(y, 60, 5);
        // client->rotateByYawRate(yaw_rate, time_to_spin);
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

coord Drone::gps()
{
	auto pos = client->getPosition();

	return {pos.x() - initial_gps.x,
        pos.y() - initial_gps.y,
        pos.z() - initial_gps.z};
}

coord Drone::position(std::string localization_method)
{
    tf::StampedTransform transform;

    try{
      tfListen.lookupTransform("/world", "/"+localization_method,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    coord result;
    auto tf_translation = transform.getOrigin();
    result.x = tf_translation.y();
    result.y = tf_translation.x();
    result.z = -tf_translation.z();

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

int Drone::collisions()
{
    auto col_info = client->getCollisionInfo();
    return col_info.collison_count;
}

