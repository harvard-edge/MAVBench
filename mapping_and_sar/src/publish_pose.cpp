#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "template_library.hpp"
#include <sstream>
//#include "rpc/RpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
//#include "controllers/DroneControllerBase.hpp"
#include "common/Common.hpp"
#include <fstream>
#include "Drone.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
void sigIntPrivateHandler(int sig)
{
    ros::shutdown();
    exit(0);
}

void control_drone(Drone& drone)
{
	cout << "Initialize drone:\n";
	cout << "\ta: arm\n";
	cout << "\td: disarm\n";
	cout << "\tt h: takeoff to h m\n";
	cout << "\tl: land\n";
	cout << "\tf x y z d: fly at (x,y,z) m/s for d s\n";
	cout << "\ty x: set yaw to x\n";
	cout << "\tp: print pitch, roll, yaw, height\n";
	cout << "\tc: complete drone setup and continue\n";
	cout << "\tCtrl-c: quit\n";

	std::string cmd("");

	while(cmd != "c") {
		cin >> cmd;

	    if (cmd == "a") {
	        drone.arm();
		} else if (cmd == "d") {
			drone.disarm();
		} else if (cmd == "t") {
			double height;
			cin >> height;
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
			drone.takeoff(height);
		} else if (cmd == "l") {
			drone.land();
		} else if (cmd == "f") {
			double x,y,z,d;
			cin >> x >> y >> z >> d;
			drone.fly_velocity(x, y, z, d);
		} else if (cmd == "y") {
			double x;
			cin >> x;
			drone.set_yaw(x);
		} else if (cmd == "p") {
			auto pos = drone.pose().position;
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " pos: " << pos.x << ", " << pos.y << ", " << pos.z << endl;
        } else if (cmd != "c") {
			cout << "Unknown command" << endl;
		}
	}
}




// *** F:DN main function
int main(int argc, char **argv)
{
    
     //ROS_ERROR("heelo"); 
    // ROS node initialization
    ros::init(argc, argv, "publish_pose", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    
    
    std::string localization_method; 
    signal(SIGINT, sigIntPrivateHandler);
 
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_topic", 10);
    uint16_t port = 41451;
    ros::param::get("/publish_pose/ip_addr",ip_addr__global);
    //ROS_ERROR_STREAM("blah"<<ip_addr__global);
    ros::Rate pub_rate(20);
    if(!ros::param::get("/publish_pose/localization_method",localization_method))  {
        ROS_FATAL_STREAM("Could not start pubslish pose cause localization_method not provided");
        return -1; 
    }
    
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    while (ros::ok())
	{
        //control_drone(drone);
        
        geometry_msgs::PoseWithCovariance  drone_pose = drone.pose_with_covariance();
        geometry_msgs::PoseWithCovarianceStamped drone_pose_stamped; 
        drone_pose_stamped.pose = drone_pose;
        //drone_pose_stamped.header.seq //this is incremented by roscp
        drone_pose_stamped.header.stamp = ros::Time::now();
        drone_pose_stamped.header.frame_id = "world"; //possible change
                
        pose_pub.publish(drone_pose_stamped);
	 	
        pub_rate.sleep();
    }
    

}
