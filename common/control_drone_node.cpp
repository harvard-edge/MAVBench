#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "template_library.hpp"
#include <sstream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
//#include "controllers/DroneControllerBase.hpp"
//#include "common/Common.hpp"
#include "common.h"
#include <fstream>
#include "Drone.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "control_drone.h"
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>
#include "common.h"
#include <cstring>
#include <string>
using namespace std;
std::string ip_addr__global;

// *** F:DN main function
int main(int argc, char **argv)
{
    
    // ROS node initialization
    ros::init(argc, argv, "control_drone_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandler);
 
    uint16_t port = 41451;
    // ros::param::get("/ip_addr",ip_addr__global);
    ip_addr__global = "10.157.90.51";
    ROS_INFO_STREAM("ip to contact to"<<ip_addr__global);
    Drone drone(ip_addr__global.c_str(), port);
	ros::Rate pub_rate(5);


    while (ros::ok())
	{
        control_drone(drone);
        pub_rate.sleep();
    }
    

}


