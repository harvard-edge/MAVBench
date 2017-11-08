#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include "api/RpcLibClient.hpp"
#include "coord.h"

//#include "configs.h"
// Control functions

class Drone {
public:
	Drone();
	Drone(const std::string& ip_addr, uint16_t port);

	~Drone();

	// *** F:DN Connection functions
	void connect();
	void connect(const std::string& ip_addr, uint16_t port);

	// *** F:DN Control functions
    void arm();
    void disarm();
    bool takeoff(double h);
    bool set_yaw(float y);
    bool fly_velocity(double vx, double vy, double vz, double duration = 3);
    bool land();
    float get_pitch();
    float get_yaw();
    float get_roll();

    // *** F:DN Localization functions
    coord position(std::string localization_method); 
    coord gps();

    int collisions();

private:
	msr::airlib::RpcLibClient * client;
    coord initial_gps;
    tf::TransformListener tfListen;
};

#endif

