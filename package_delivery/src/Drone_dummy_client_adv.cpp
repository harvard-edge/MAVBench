#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "template_library.hpp"
#include <sstream>
#include "rpc/RpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
//#include "controllers/DroneControllerBase.hpp"
#include "common/Common.hpp"
#include <fstream>
#include "Drone.h"
#include "package_delivery/get_trajectory.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>



using namespace std;


void control_drone(Drone& drone, string cmd)
{
//	cout << "Initialize drone:\n";
//	cout << "\ta: arm\n";
//	cout << "\td: disarm\n";
//	cout << "\tt h: takeoff to h m\n";
//	cout << "\tl: land\n";
//	cout << "\tf x y z d: fly at (x,y,z) m/s for d s\n";
//	cout << "\ty x: set yaw to x\n";
//	cout << "\tp: print pitch, roll, yaw, height\n";
//	cout << "\tc: complete drone setup and continue\n";
//	cout << "\tq: quit\n";
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
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " height: " << drone.gps().z << endl;
		} else if (cmd == "q") {
			exit(0);
		} else if (cmd != "c") {
			cout << "Unknown command" << endl;
		}
	
}

void control_drone_velocity(Drone& drone, std::vector<float> velocity_vec)
{
    float x = velocity_vec[0];
    float y = velocity_vec[1];
    float z = velocity_vec[2];
    float d = velocity_vec[3];
    
    drone.fly_velocity(x, y, z, d);
}




/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

	ros::init(argc, argv, "Drone_dummy_client_adv_node");
	if (argc != 1)
	{
		ROS_INFO("usage: add_two_ints_client X Y");
		return 1;
	}

	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<package_delivery::get_trajectory>("dummy_service_adv");
	package_delivery::get_trajectory srv;
    
    // *** F:DN hardcoding the goal 
    geometry_msgs::Point goal;
    goal.x = 1;
    goal.y = 2;
    goal.z = 3; 
    srv.request.goal = goal;
	
    ros::Rate loop_rate(100);

	// *** F:DN defining a communication input_sample object for communication
	string ip_addr = "10.157.34.101";
	uint16_t port = 41451;
	Drone Drone__obj(ip_addr.c_str(), port);
	int count = 0;
    //srv.request.goal = "10";
	
    while (ros::ok())
	{
        if (client.call(srv))
		{
            ROS_INFO("recieve control signal:");
            //ROS_INFO("recieve control signal: [%s]", srv.response.ctrl.c_str());
		}
		else
		{
			ROS_ERROR("Failed to call service add_two_ints");
		}

		for (auto it = srv.response.multiDOFtrajectory.points.begin(); it != srv.response.multiDOFtrajectory.points.end(); it++) { 
            std::vector<float> velocity;
            velocity.push_back(it->velocities[0].linear.x);
            velocity.push_back(it->velocities[0].linear.y);
            velocity.push_back(it->velocities[0].linear.z);
            double secs = it->time_from_start.toSec();
            control_drone_velocity(Drone__obj, velocity);
        } 
        

		loop_rate.sleep();
		++count;
	}

	return 0;
}
