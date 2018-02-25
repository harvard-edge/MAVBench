#ifndef DRONE_H
#define DRONE_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <stdint.h>
#include <string>
#include <limits>
#include <opencv2/opencv.hpp>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <geometry_msgs/Pose.h>
#include "coord.h"
#include "common/VectorMath.hpp"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "coord.h"

//#include "configs.h"
// Control functions

const float FACE_FORWARD = std::numeric_limits<float>::infinity();
const float FACE_BACKWARD = -std::numeric_limits<float>::infinity();
const float YAW_UNCHANGED = -1e9;

class Drone {
public:
    Drone();
    Drone(const std::string& ip_addr, uint16_t port);
    Drone(const std::string& ip_addr, uint16_t port, std::string localization_method);
    
    ~Drone();

    // *** F:DN Connection functions
    void connect();
    void connect(const std::string& ip_addr, uint16_t port);
    void set_localization_method(std::string localization_method);

	// *** F:DN Control functions
    void arm();
    void disarm();
    bool takeoff(double h);
    bool set_yaw(int y);
    bool fly_velocity(double vx, double vy, double vz, float yaw = YAW_UNCHANGED, double duration = 3);
    bool land();
    bool set_yaw_based_on_quaternion(geometry_msgs::Quaternion q);

    // *** F:DN Localization functions
    coord position(); 
    geometry_msgs::Pose pose();
    geometry_msgs::PoseWithCovariance pose_with_covariance();
    float get_pitch();
    float get_yaw();
    float get_roll();
    //coord gps();

    // *** F:DN Stats functions
    msr::airlib::FlightStats getFlightStats();
    
    msr::airlib::IMUStats getIMUStats();
    
    // *** F:DN Collison functions
    msr::airlib::CollisionInfo getCollisionInfo();

    // *** F:DN Drone parameters functions
    float maxYawRate();
    float maxYawRateDuringFlight();

private:
    msr::airlib::MultirotorRpcLibClient * client;

    std::string localization_method; 
    tf::TransformListener tfListen;

    uint64_t collision_count;

    float max_yaw_rate = 15.0;
    //float max_yaw_rate_during_flight = 90.0;
    float max_yaw_rate_during_flight = 10.0;
    // Initial position as determined by the flight-controller 
    coord initial_fc_pos;
};

#endif

