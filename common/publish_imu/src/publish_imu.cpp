#include "ros/ros.h"
#include <std_msgs/String.h>
//#include "template_library.hpp"
#include <sstream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "controllers/DroneControllerBase.hpp"
//#include "common/Common.hpp"
#include "common.h"
#include <fstream>
#include "Drone.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>
#include "common.h"
#include <cstring>
#include <string>
using namespace std;
std::string ip_addr__global;
using namespace msr::airlib;

// *** F:DN main function
int main(int argc, char **argv)
{
    
    // ROS node initialization
    ros::init(argc, argv, "publish_imu", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
    std::string ns = ros::this_node::getName();
    uint16_t port = 41451;
    if (!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL("Could not start mapping. Parameter missing! Looking for %s",
                (ns + "/ip_addr").c_str());
        return -1;
    }
    Drone drone(ip_addr__global.c_str(), port);
	ros::Rate pub_rate(40);
    sensor_msgs::Imu IMU_msg;
    ros::Publisher IMU_pub = nh.advertise <sensor_msgs::Imu>("imu_topic", 1);
    IMUStats IMU_stats;
    //geometry_msgs::Vector3 linear_acceleration; 
    //geometry_msgs::Vector3 angular_velocity; 
    //geometry_msgs::Quaternion orientation;     
    //float roll, yaw, pitch;
    //msr::airlib::VectorMath::toEulerianAngle(IMU_stats.orientation, pitch, roll, yaw);
    //ROS_INFO_STREAM(yaw*180/M_PI);

    while (ros::ok())
	{
        //publish(drone);
        IMU_stats = drone.getIMUStats();  
        
        IMU_msg.orientation.x  = IMU_stats.orientation.x();
        IMU_msg.orientation.y  = IMU_stats.orientation.y();
        IMU_msg.orientation.z  = IMU_stats.orientation.z();
        IMU_msg.orientation.w  = IMU_stats.orientation.w();
        IMU_msg.orientation_covariance[0] = .00001;
        IMU_msg.orientation_covariance[4] = .00001;
        IMU_msg.orientation_covariance[8] = .00001;

        IMU_msg.angular_velocity.x = IMU_stats.angular_velocity[0];
        IMU_msg.angular_velocity.y = IMU_stats.angular_velocity[1];
        IMU_msg.angular_velocity.z = IMU_stats.angular_velocity[2];
        IMU_msg.angular_velocity_covariance[0] = .00001;
        IMU_msg.angular_velocity_covariance[4] = .00001;
        IMU_msg.angular_velocity_covariance[8] = .00001;

        IMU_msg.linear_acceleration.x = IMU_stats.linear_acceleration[0];
        IMU_msg.linear_acceleration.y = IMU_stats.linear_acceleration[1];
        IMU_msg.linear_acceleration.z = IMU_stats.linear_acceleration[2];
        IMU_msg.linear_acceleration_covariance[0] = .00001;
        IMU_msg.linear_acceleration_covariance[4] = .00001;
        IMU_msg.linear_acceleration_covariance[8] = .00001;

        IMU_pub.publish(IMU_msg);
        pub_rate.sleep();
    }
    

}


