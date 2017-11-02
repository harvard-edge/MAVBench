#include "Drone.h"
#include <iostream>
#include <vector>
#include <cstring>
#include "common/Common.hpp"
#include "coord.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

Drone::Drone() : client(0)
{
    this->control_drone = false;
    connect();
}

Drone::Drone(const std::string& ip_addr, uint16_t port, bool control_drone) : client(0)
{
    this->control_drone = control_drone;	
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
	client = new msr::airlib::MultirotorRpcLibClient();
    client->enableApiControl(control_drone);
}

void Drone::connect(const std::string& ip_addr, uint16_t port)
{
	if (client != 0)
		delete client;
	client = new msr::airlib::MultirotorRpcLibClient(ip_addr, port);
    ROS_INFO_STREAM("this control"<<this->control_drone); 
    client->enableApiControl(this->control_drone);
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
	const double takeoff_timeout = 20.0;
    auto ground_pos = client->getPosition();

	try {
		client->takeoff(takeoff_timeout);
	} catch (...) {
		std::cout << "Taking off failed" << std::endl;
	}

	std::cout << "Press enter to set offboard mode";
	while (std::cin.get() != '\n') {}

	try {
		//client->setOffboardMode(true);
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
		//client->moveByAngle(y, float(60), float(5),this->gps().z,1);
        //client->moveByAngle(0, 0, this->gps().z, y, 3);
    }catch(...){
		std::cerr << "set_yaw failed" << std::endl;
		return false;
	}
	return true;
}


bool Drone::set_yaw_based_on_quaternion(geometry_msgs::Quaternion q)
{
	float pitch, roll, yaw;
    Eigen::Quaternion<float,Eigen::DontAlign> q_airsim_style;
    q_airsim_style.x() = q.x;
    q_airsim_style.y() = q.y;
    q_airsim_style.z() = q.z;
    q_airsim_style.w() = q.w;
    msr::airlib::VectorMath::toEulerianAngle(q_airsim_style, pitch, roll, yaw);
	
    try {
        this->set_yaw(yaw*180 / M_PI);
    } catch(...) {
		std::cerr << "set_yaw_based_on_quaternion failed" << std::endl;
		return false;
	}
	return true;
}




bool Drone::fly_velocity(double vx, double vy, double vz, double duration)
{
	try {
		client->moveByVelocity(vy,vx, -1*vz, duration);
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

coord Drone::gps_airsim()
{
	auto pos = client->getPosition();
	return {pos.x(), pos.y(), pos.z()};
}

coord Drone::gps()
{
	auto pos = client->getPosition();
	return { pos.y(), pos.x(), -1*pos.z()};
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


geometry_msgs::Pose Drone::get_geometry_pose(){
    geometry_msgs::Pose pose;
	auto q = client->getOrientation();
	//auto p = client->getPosition();
    pose.position.x = client->getPosition().y();
    pose.position.y = client->getPosition().x();
    pose.position.z = -1*client->getPosition().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}


geometry_msgs::PoseWithCovariance Drone::get_geometry_pose_with_coveraiance(){
    geometry_msgs::PoseWithCovariance pose_with_covariance;
    geometry_msgs::Pose pose;
	
    auto q = client->getOrientation();
    pose.position.x = client->getPosition().y();
    pose.position.y = client->getPosition().x();
    pose.position.z = -1*client->getPosition().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_with_covariance.pose = pose;
    for (int i = 0; i <36; i++) { 
        //https://answers.ros.org/question/181689/computing-posewithcovariances-6x6-matrix/ 
        pose_with_covariance.covariance[i] = 0;
    } 
    return pose_with_covariance;
}




float Drone::get_pitch()
{
	auto q = client->getOrientation();
	float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return p*180 / M_PI;
}
