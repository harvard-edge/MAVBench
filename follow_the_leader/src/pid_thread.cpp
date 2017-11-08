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
#include "pid.h"

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>
#include "common.h"
#include <cstring>
#include <string>
#include "follow_the_leader/bounding_box_msg.h"
#include "bounding_box.h"
#include "follow_the_leader/cmd_srv.h"

using namespace std;
std::string ip_addr__global;
std::queue<bounding_box> bb_queue; //uesd to buffer imags while detection is running
//const int image_w = 256, image_h = 144; //this must be equal to the img being pulled in from airsim
const int image_w = 400, image_h = 400; //this must be equal to the img being pulled in from airsim
float vx__K = (float)2.0/(image_h/2); 
float vy__K = (float)3.0/(image_w/2); 
float vz__K = (float)1.0;



void sigIntHandler(int sig)
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
			auto pos = drone.gps();
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " pos: " << pos.x << ", " << pos.y << ", " << pos.z << endl;
        } else if (cmd == "s") {
            spin_around(drone); 
        }else if (cmd != "c") {
			cout << "Unknown command" << endl;
		}
	}
}

void bb_cb(const follow_the_leader::bounding_box_msg::ConstPtr& msg) {
    bounding_box bb;
    bb.x = msg->x;
    bb.y = msg->y;
    bb.w = msg->w;
    bb.h = msg->h;
    bb.conf  = msg->conf;
    bb_queue.push(bb);
}

// *** F:DN main function

void fly_towards_target(Drone& drone, const bounding_box& bb,
        int img_height, int img_width, PID& pid_vx, PID& pid_vy, PID& pid_vz,
        double dt)
{
	static float hover_height = drone.gps().z;
	const float height_ratio = 0.3;

	auto yaw = drone.get_yaw();
	if (yaw > 15 || yaw < -15) {
		cout << "Correcting yaw\n";
		drone.set_yaw(0);
	}
	
    double bb__cntr__x =  bb.x + bb.w/2;
    double bb__cntr__y =  bb.y + bb.h/2;
    
    double img__cntr =  img_width / 2;
    //ROS_INFO_STREAM("bb h is "<<bb.h<< " height ration is"<<height_ratio*img_height);	
    ROS_INFO_STREAM("result for vy"<<"bb.h"<<bb.h);
    double vy = pid_vy.calculate(bb.h, height_ratio*img_height,  dt); //get closer to the person
    //ROS_INFO_STREAM("result for vx");
    double vx = pid_vx.calculate(img__cntr, bb__cntr__x, dt); //keep the person in the center of the img 
    //ROS_INFO_STREAM("result for vz");
    double vz = pid_vz.calculate(drone.gps().z, hover_height, dt); //for hovering at the same point

    //ROS_INFO_STREAM("velocities"<<vx<< " " << vy<< " " << vz);	
    drone.fly_velocity(vx, vy, vz);
}



int main(int argc, char **argv)
{
    
    // ROS node initialization
    ros::init(argc, argv, "pid_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
 
    uint16_t port = 41451;
    ros::param::get("/pid_node/ip_addr",ip_addr__global);
    //ROS_ERROR_STREAM("blah"<<ip_addr__global);
    Drone drone(ip_addr__global.c_str(), port);
	ros::Rate pub_rate(10);
    ros::Subscriber bb_sub = nh.subscribe("/bb_topic", 4, bb_cb);
    control_drone(drone);
    PID pid_vx(vx__K, 0, 0, 2, -2);
    PID pid_vy(vy__K, 0, 0, 2, -2);
	PID pid_vz(vz__K, 0, 0, 0.5, -0.5); //at the moment only for keeping drone stable


    while (ros::ok())
	{
        while(!bb_queue.empty()) {
            ROS_INFO_STREAM("queue size"<<bb_queue.size()); 
            auto bb = bb_queue.front(); 
            bb_queue.pop(); 
            fly_towards_target(drone, bb, image_h, image_w, pid_vx, pid_vy, pid_vz, 1); // dt is not currently used
        }
        ros::spinOnce(); 
        pub_rate.sleep();
    }
    

}
