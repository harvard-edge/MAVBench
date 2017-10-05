#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "api/RpcLibClient.hpp"
#include <geometry_msgs/Pose.h>
#include "coord.h"

//#include "configs.h"
// Control functions

class Drone {
public:
	Drone();
	Drone(const std::string& ip_addr, uint16_t port);

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

private:
	msr::airlib::RpcLibClient * client;
};

#endif
