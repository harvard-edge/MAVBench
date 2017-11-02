#ifndef DRONE_H
#define DRONE_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <stdint.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
//#include "common/common.hpp"
#include "common/VectorMath.hpp"
#include <geometry_msgs/Pose.h>
#include "coord.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>


//#include "configs.h"
// Control functions

class Drone {
public:
	Drone();
	Drone(const std::string& ip_addr, uint16_t port, bool control_drone=false);

	~Drone();

	// *** F:DN Control functions
	void connect();
	void connect(const std::string& ip_addr, uint16_t port);

	// *** F:DN Camera functions
    void arm();
    void disarm();
    bool takeoff(double h);
    bool set_yaw(float y);
    bool fly_velocity(double vx, double vy, double vz, double duration = 3);
    bool land();
    float get_pitch();
    float get_yaw();
    float get_roll();
    coord gps();
    coord gps_airsim();
    geometry_msgs::Pose get_geometry_pose();
    geometry_msgs::PoseWithCovariance get_geometry_pose_with_coveraiance();

    bool set_yaw_based_on_quaternion(geometry_msgs::Quaternion q);
private:
   bool control_drone;	
    msr::airlib::MultirotorRpcLibClient * client;
};

#endif
