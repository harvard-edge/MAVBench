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
#include "Profiling.h"
#include <cstring>
#include <string>
#include "HelperFunctions/QuatRotEuler.h"
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
    
    int samples = 0;
    int misses = 0;

    ros::Rate pub_rate(110);
    sensor_msgs::Imu IMU_msg;
    ros::Publisher IMU_pub = nh.advertise <sensor_msgs::Imu>("imu_topic", 1);
    ros::Publisher pose_pub = nh.advertise <geometry_msgs::Pose>("pose_topic", 1);
    
    IMUStats IMU_stats;
    uint64_t last_t = 0;
    //geometry_msgs::Vector3 linear_acceleration; 
    //geometry_msgs::Vector3 angular_velocity; 
    //geometry_msgs::Quaternion orientation;     
    //float roll, yaw, pitch;
    //msr::airlib::VectorMath::toEulerianAngle(IMU_stats.orientation, pitch, roll, yaw);
    //ROS_INFO_STREAM(yaw*180/M_PI);
    
    ROS_WARN("Hey!");

    while (ros::ok())
	{
        //publish(drone);
        IMU_stats = drone.getIMUStats();  

	geometry_msgs::Quaternion q = setQuat(IMU_stats.orientation.x(),
                                              IMU_stats.orientation.y(),
                                              IMU_stats.orientation.z(),
                                              IMU_stats.orientation.w());
	/*
	 geometry_msgs::Vector3 rpy = quat2rpy(q);
	// std::cout << rpy << std::endl;
	 rpy.y = -rpy.y;
	 rpy.z = -rpy.z + M_PI/2.0;

	 geometry_msgs::Quaternion q_body2cam = setQuat(0.5, -0.5, 0.5, -0.5);
	 geometry_msgs::Quaternion q_cam = rpy2quat(rpy);
	 q_cam = quatProd(q_body2cam, q_cam);
	 tf::Quaternion qOrientation(q_cam.x, q_cam.y, q_cam.z, q_cam.w);
*/
	tf::Quaternion qOrientation(q.x, q.y, q.z, q.w);
	
	tf::Matrix3x3 rotationMatrix(qOrientation);
	tf::Matrix3x3 rotationMatrixTransposed = rotationMatrix.transpose();
	tf::Vector3 accelerationWorld(IMU_stats.linear_acceleration[0], IMU_stats.linear_acceleration[1], IMU_stats.linear_acceleration[2]);

	
	tf::Transform tfMatrix(qOrientation);
//	tf::Vector3 accelerationBody = tfMatrix * accelerationWorld;

	tf::Vector3 accelerationBody =  accelerationWorld;


	// Eigen::Vector3d accelerationBody;
	// accelerationBody[0] = rotationMatrixTransposed[0][0] * accelerationWorld[0] + rotationMatrixTransposed[0][1] * accelerationWorld[1] + rotationMatrixTransposed[0][2] * accelerationWorld[2];
	// accelerationBody[1] = rotationMatrixTransposed[1][0] * accelerationWorld[0] + rotationMatrixTransposed[1][1] * accelerationWorld[1] + rotationMatrixTransposed[1][2] * accelerationWorld[2];
	// accelerationBody[2] = rotationMatrixTransposed[2][0] * accelerationWorld[0] + rotationMatrixTransposed[2][1] * accelerationWorld[1] + rotationMatrixTransposed[2][2] * accelerationWorld[2];
//	std::cout<<"\n--------------"<<std::endl;
 //       std::cout << "a[0]: "<<rotationMatrix[0][0] << " * " << accelerationWorld[0] << " + " << rotationMatrix[0][1] << " * " << accelerationWorld[1] << " + " << rotationMatrix[0][2] << "*" << accelerationWorld[2] << " = " << accelerationBody[0] << std::endl;

//	 std::cout << "a[1]: "<< rotationMatrix[1][0] << " * " << accelerationWorld[0] << " + " << rotationMatrix[1][1] << " * " << accelerationWorld[1] << " + " << rotationMatrix[1][2] << "*" << accelerationWorld[2] << " = " << accelerationBody[1] << std::endl;

//	 std::cout << "a[2]: "<< rotationMatrix[2][0] << " * " << accelerationWorld[0] << " + " << rotationMatrix[2][1] << " * " << accelerationWorld[1] << " + " << rotationMatrix[2][2] << "*" << accelerationWorld[2] << " = " << accelerationBody[2] << std::endl;

/*
	 std::cout << "Body:"<< 
			      accelerationBody[0] << " * " << accelerationBody[0]  << " + " <<
			      accelerationBody[1] << " * " << accelerationBody[1]  << " + "  <<
			      accelerationBody[2] << " * " << accelerationBody[2]  << " = "  <<
			      accelerationBody[0]*accelerationBody[0] + 
			      accelerationBody[1]*accelerationBody[1] + 
			      accelerationBody[2]*accelerationBody[2] <<std::endl;

	 std::cout << "World:"<< 
				accelerationWorld[0] << " * " << accelerationWorld[0]  << " + " <<
				accelerationWorld[1] << " * " << accelerationWorld[1]  << " + " <<
				accelerationWorld[2] << " * " << accelerationWorld[2]  << " = " <<
				accelerationWorld[0]*accelerationWorld[0] + 
				accelerationWorld[1]*accelerationWorld[1] + 
				accelerationWorld[2]*accelerationWorld[2] <<std::endl;
*/
// if (IMU_stats.linear_acceleration[0] >= 2) {
	// 	return 0;
	// }

        IMU_msg.orientation.x = IMU_stats.orientation.x();
        IMU_msg.orientation.y = IMU_stats.orientation.y();
        IMU_msg.orientation.z = IMU_stats.orientation.z();
        IMU_msg.orientation.w = IMU_stats.orientation.w();
        IMU_msg.orientation_covariance[0] = 0.1;
        IMU_msg.orientation_covariance[4] = 0.1;
        IMU_msg.orientation_covariance[8] = 0.1;

        IMU_msg.angular_velocity.x = IMU_stats.angular_velocity[0];
        IMU_msg.angular_velocity.y = IMU_stats.angular_velocity[1];
        IMU_msg.angular_velocity.z = IMU_stats.angular_velocity[2];
        IMU_msg.angular_velocity_covariance[0] = 0.1;
        IMU_msg.angular_velocity_covariance[4] = 0.1;
        IMU_msg.angular_velocity_covariance[8] = 0.1;

	// std::cout << "-------------\n";
	// std::cout << IMU_stats.linear_acceleration << std::endl;
        // std::cout <<accelerationBody[0] << " \t" << accelerationBody[1]<< "\t" << accelerationBody[2]<<std::endl;
 	// 
	// std::cout << "\n\n";
	// std::cout << rotationMatrix[0][0] <<"\t" << rotationMatrix[0][1] <<"\t" <<rotationMatrix[0][2] << std::endl;
	// std::cout << rotationMatrix[1][0] <<  "\t" << rotationMatrix[1][1] << "\t"<<rotationMatrix[1][2] << std::endl;
	// std::cout << rotationMatrix[2][0] << "\t"<< rotationMatrix[2][1] << "\t"<< rotationMatrix[2][2] << std::endl;
       
 
	IMU_msg.linear_acceleration.x = accelerationBody[0]; // IMU_stats.linear_acceleration[0];
        IMU_msg.linear_acceleration.y = accelerationBody[1]; // IMU_stats.linear_acceleration[1];
        IMU_msg.linear_acceleration.z = accelerationBody[2] ; // IMU_stats.linear_acceleration[2];
        IMU_msg.linear_acceleration_covariance[0] = 0.1;
        IMU_msg.linear_acceleration_covariance[4] = 0.1;
        IMU_msg.linear_acceleration_covariance[8] = 0.1;

        IMU_msg.header.stamp = ros::Time(uint32_t(IMU_stats.time_stamp / 1000000000 ), uint32_t(IMU_stats.time_stamp % 1000000000));
	// IMU_msg.header.stamp = ros::Time::now();
    
    /*if (IMU_stats.time_stamp != IMU_stats.after_time_stamp) {
        ROS_ERROR("Uh oh!");
    }*/

	samples++;
    if (last_t < IMU_stats.time_stamp) {
		last_t = IMU_stats.time_stamp;
		IMU_pub.publish(IMU_msg);
	} else {
		/*if (last_t == IMU_stats.time_stamp)
			std::cout << last_t << " = " << IMU_stats.time_stamp << std::endl;
		else
			std::cout << last_t << " > " << IMU_stats.time_stamp << std::endl;
		*/
		misses++;
		// std::cout << (misses*100) / samples << "%\n";
	}

    geometry_msgs::Pose pose;
    pose.position.x = drone.pose().position.x;
    pose.position.y = drone.pose().position.y;
    pose.position.z = drone.pose().position.z;
    pose_pub.publish(pose);

	// std::cout << IMU_stats.time_stamp << "\t" << IMU_msg.header.stamp.toSec() << std::endl;
	// std::cout << "\t" << ros::Time::now().toSec() << std::endl;
        pub_rate.sleep();
    }
}

