#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <signal.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "common.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/default_topics.h>
#include "Drone.h"
#include <iostream>
#include <list>

using namespace std;

std::string ip_addr__global;
std::string mav_name;
list<trajectory_msgs::MultiDOFJointTrajectoryPoint> traj_combined_pt_list;
trajectory_msgs::MultiDOFJointTrajectoryPoint last_p;
bool first_point_ever;


/*
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
*/
void sigIntHandler(int sig)
{
    ros::shutdown();
    exit(0);
}

void callback_trajactory(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)//, Drone drone)//, bool &too_close)
{
    for (int i = 0; i < msg->points.size(); ++i) {
        //ROS_INFO_STREAM("i is "<< i);
        traj_combined_pt_list.push_back(msg->points[i]);
        
        //ROS_INFO_STREAM("PUSHING pt"<<msg->points[i].transforms[0].translation.x << " " <<msg->points[i].transforms[0].translation.y << " "
         //       <<msg->points[i].transforms[0].translation.z );
      
    }     
    /*
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port);
    double segment_dedicated_time = .8;
    ROS_ERROR_STREAM("in callback_trajactory "<<msg->points.size());
    
    for (int i = 0; i < msg->points.size()-1; ++i) {
        ROS_ERROR("in callback_trajactory");
        auto p = msg->points[i];
        auto p_next = msg->points[i+1];
        double v_x = (p.transforms[0].translation.x -  p_next.transforms[0].translation.x)/segment_dedicated_time;
        double v_y = (p.transforms[0].translation.y -  p_next.transforms[0].translation.y)/segment_dedicated_time;
        double v_z = (p.transforms[0].translation.z -  p_next.transforms[0].translation.z)/segment_dedicated_time;
        auto segment_start_time = std::chrono::system_clock::now();
         
        drone.fly_velocity(v_x, 
                v_y,
                v_z + 0.2*(p.transforms[0].translation.z-drone.gps().z));
        drone.set_yaw_based_on_quaternion(p.transforms[0].rotation); 
         
        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>((1)*segment_dedicated_time));

    }
    */
}



//for (int i = 0; i < msg->points.size()-1; ++i) {
void take_commands_and_follow(list<trajectory_msgs::MultiDOFJointTrajectoryPoint> &traj_combined_pt_list, Drone &drone) {
    //double segment_dedicated_time = 2;
    double yaw_t; 
    double dt; 
    std::string ns = ros::this_node::getName();
    //ros::param::get("/follow_trajectory/yaw_t",yaw_t);
    if (!ros::param::get(ns + "/follow_trajectory/yaw_t", yaw_t)) {
        ROS_FATAL_STREAM("Could not start exploration. Parameter missing! Looking for"<<
                "/follow_trajectory/yaw_t");
        return;
    }
    
    if (!ros::param::get(ns + "/nbvp/dt", dt)) {
        ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
                (ns + "/nbvp/dt").c_str());
        return;
    }

    //if (!ros::param::get("/follow_trajectory/segment_dedicated_time",segment_dedicated_time)){
    double segment_dedicated_time = yaw_t + dt;
    //ROS_INFO_STREAM("segment_dedicated_time"<<segment_dedicated_time); 
    /*
    ros::param::get("/follow_trajectory/segment_dedicated_time",segment_dedicated_time);
    if (!ros::param::get("/follow_trajectory/segment_dedicated_time",segment_dedicated_time)){
    ROS_FATAL_STREAM("Could not start exploration. Parameter missing! Looking for"<<
              "/follow_trajectory/segment_dedicated_time");
    return;
  }
*/
    //ROS_INFO("insdie follow trajectory");
    if (traj_combined_pt_list.empty() ) {//*** F:DN nothing in the list, just return
        return;
    }else if (first_point_ever) {//if first point, still can't get a velocity out of it
       first_point_ever = false; 
       auto p = traj_combined_pt_list.front();  
       traj_combined_pt_list.pop_front(); 
       last_p = p;
       return; 
    }   
        
    while(!traj_combined_pt_list.empty()){ 
        //trajectory_msgs::MultiDOFJointTrajectoryPoint point = trajectory_pt_list.back();
        //auto p = g.points[i];
        //auto p = trajectory_combined_pt_list.back();  
        //auto p_next = msg.points[i+1];
        auto segment_start_time = std::chrono::system_clock::now();
        auto cur_p = traj_combined_pt_list.front();  
        traj_combined_pt_list.pop_front(); 

        double v_x = (cur_p.transforms[0].translation.x -  last_p.transforms[0].translation.x)/segment_dedicated_time;
        double v_y = (cur_p.transforms[0].translation.y -  last_p.transforms[0].translation.y)/segment_dedicated_time;
        double v_z = (cur_p.transforms[0].translation.z - last_p.transforms[0].translation.z)/segment_dedicated_time;
        //ROS_INFO_STREAM("yaw tobe set"<< cur_p.transforms[0].rotation);
        //ROS_INFO("now"); 
        drone.set_yaw_based_on_quaternion(cur_p.transforms[0].rotation); 
        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>(1));
        //ROS_INFO("after now"); 
        //ROS_INFO_STREAM("--------");
        //ROS_INFO_STREAM("last_p x,y,z"<< last_p.transforms[0].translation.x<<" "<< last_p.transforms[0].translation.y<< " " << last_p.transforms[0].translation.z);
        //ROS_INFO_STREAM("cur_p x,y,z"<< cur_p.transforms[0].translation.x<<" "<< cur_p.transforms[0].translation.y<< " " << cur_p.transforms[0].translation.z);
        //ROS_INFO_STREAM("v x,y,z"<< v_x<< " " << v_y<<" "<< v_z);
        
        drone.fly_velocity(v_x + 0.2*(cur_p.transforms[0].translation.x-drone.pose().position.x),
                v_y + 0.2*(cur_p.transforms[0].translation.y-drone.pose().position.y),
                v_z + 0.2*(cur_p.transforms[0].translation.z-drone.pose().position.z), segment_dedicated_time);
        
        /* 
        drone.fly_velocity(v_x, 
                v_y,
                v_z, segment_dedicated_time);
        */
        last_p = cur_p;
        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>((1)*segment_dedicated_time));
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "follow_trajectory", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandler);
    std::string localization_method; 
    ros::param::get("/follow_trajectory/ip_addr",ip_addr__global);
    ros::param::get("/follow_trajectory/mav_name",mav_name);
    if(!ros::param::get("/package_delivery/localization_method",localization_method))  {
      ROS_FATAL_STREAM("Could not start search and rescue cause localization_method not provided");
    return -1; 
    }
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    //double segment_dedicated_time = 1;
    first_point_ever = true;

    string topic_name =  mav_name + "/" + mav_msgs::default_topics::COMMAND_TRAJECTORY;
    ros::Subscriber trajectory_follower_sub = n.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(topic_name, 100, callback_trajactory);
    
    
    //ros::Subscriber trajectory_follower_sub = n.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 100, boost::bind(callback_trajactory, _1, drone));
    //ros::Subscriber trajectory_follower_sub = n.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 100, boost::bind(callback_trajactory, _1, 1));

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce(); 	
        take_commands_and_follow(traj_combined_pt_list, drone); 
        //if (trajectory_list. 
        loop_rate.sleep();
    }
    return 0;
}
