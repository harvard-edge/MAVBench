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
#include "follow_the_leader/bounding_box_msg.h"
#include "bounding_box.h"
#include "follow_the_leader/cmd_srv.h"

using namespace std;
std::string ip_addr__global;
std::string localization_method;
std::queue<bounding_box> bb_queue; //uesd to buffer imags while detection is running
float height_ratio;
//const int image_w = 256, image_h = 144; //this must be equal to the img being pulled in from airsim
int image_w__global;// = 400;
int  image_h__global; //= 400; //this must be equal to the img being pulled in from airsim
float vx__P__global; //= (float)2.0/(image_h/2); 
float vy__P__global; //= (float)3.0/(image_w/2); 
float vz__P__global; //= (float)1.0;

float vx__I__global; //= .05;
float vy__I__global; //= .05;
float vz__I__global; //= .05;

float vx__D__global; //= .1;
float vy__D__global; //= .1;
float vz__D__global; //= .1;


void bb_cb(const follow_the_leader::bounding_box_msg::ConstPtr& msg) {
    bounding_box bb;
    bb.x = msg->x;
    bb.y = msg->y;
    bb.w = msg->w;
    bb.h = msg->h;
    bb.conf  = msg->conf;
    bb_queue.push(bb);
    ROS_INFO_STREAM("height and width"<<bb.w << " " <<bb.h);
}

// *** F:DN main function

void fly_towards_target(Drone& drone, const bounding_box& bb,
        int img_height, int img_width, PID& pid_vx, PID& pid_vy, PID& pid_vz,
        double dt)
{
	static float hover_height = drone.pose().position.z;

	auto yaw = drone.get_yaw();
	if (yaw > 15 || yaw < -15) {
		cout << "Correcting yaw\n";
		drone.set_yaw(0);
	}
	
    double bb__cntr__x =  bb.x + bb.w/2;
    double bb__cntr__y =  bb.y + bb.h/2;
    
    double img__cntr =  img_width / 2;
    //ROS_INFO_STREAM("bb h is "<<bb.h<< " height ration is"<<height_ratio*img_height);	
    double vy = pid_vy.calculate(bb.h, height_ratio*img_height,  dt); //get closer to the person
    ROS_INFO_STREAM("bb.h:"<<bb.h<<" -ideal:"<< height_ratio*img_height<<" -vy:"<<vy);
    //ROS_INFO_STREAM("result for vx");
    double vx = pid_vx.calculate(img__cntr, bb__cntr__x, dt); //keep the person in the center of the img 
    //ROS_INFO_STREAM("result for vz");
    double vz = pid_vz.calculate(drone.pose().position.z, hover_height, dt); //for hovering at the same point

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

    if(!ros::param::get("/pid_node/ip_addr__global",ip_addr__global) ||
            !ros::param::get("/pid_node/vx__P__global",vx__P__global)||
            !ros::param::get("/pid_node/vy__P__global",vy__P__global)||
            !ros::param::get("/pid_node/vz__P__global",vz__P__global)||

            !ros::param::get("/pid_node/vx__I__global",vx__I__global)||
            !ros::param::get("/pid_node/vy__I__global",vy__I__global)||
            !ros::param::get("/pid_node/vz__I__global",vz__I__global)||

            !ros::param::get("/pid_node/vx__D__global",vx__D__global)||
            !ros::param::get("/pid_node/vy__D__global",vy__D__global)||
            !ros::param::get("/pid_node/vz__D__global",vz__D__global)||

            !ros::param::get("/pid_node/image_w__global",image_w__global)||
            !ros::param::get("/pid_node/image_h__global",image_h__global)||
            !ros::param::get("/localization_method",localization_method)||
            !ros::param::get("/pid_node/height_ratio",height_ratio)
            ){
        ROS_FATAL("you did not specify all the parameters");
        return -1; 
    }
            //ROS_ERROR_STREAM("blah"<<ip_addr__global);
    int loop_rate = 10; 
    float dt = ((float)1)/(float)loop_rate;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
	ros::Rate pub_rate(loop_rate);
    ros::Subscriber bb_sub = nh.subscribe("/bb_topic", 4, bb_cb);
    PID pid_vx(vx__P__global, vx__I__global, vx__D__global, 4, -4);
    PID pid_vy(vy__P__global, vy__I__global, vy__D__global, 4, -4);
	PID pid_vz(vz__P__global, vz__I__global, vz__D__global, 0.5, -0.5); //at the moment only for keeping drone stable


    while (ros::ok())
	{
        while(!bb_queue.empty()) {
            ROS_INFO_STREAM("queue size"<<bb_queue.size()); 
            auto bb = bb_queue.front(); 
            bb_queue.pop(); 
            fly_towards_target(drone, bb, image_h__global, image_w__global, pid_vx, pid_vy, pid_vz, dt); // dt is not currently used
        }
        ros::spinOnce(); 
        pub_rate.sleep();
    }
    

}
