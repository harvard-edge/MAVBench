#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>
#include <limits>
#include <fstream>
#include <mutex>
#include <atomic>
#include <stdint.h>
#include <math.h>
#include <signal.h> // To catch sigint
#include "follow_the_leader/bounding_box_msg.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "control_drone.h"
#include "Drone.h"
#include "objdetect.h"
#include "track.h"
#include "pid.h"
#include "string"
#include "configs.h"
#include "misc.h"
#include "log__class.h"
#include "follow_the_leader/cmd_srv.h"
#include "common.h"
//std::string stats_file_addr;
string status;
string ns;

using namespace std;
std::string ip_addr__global;

bool resume_detection_server_cb(follow_the_leader::cmd_srv::Request &req, 
    follow_the_leader::cmd_srv::Response &res){
    ROS_INFO_STREAM("resume detection was called");
    status = "resume_detection";
    return true;
}

int main(int argc, char** argv)
{
    //std_msgs::Bool panic_msg;
    ros::init(argc, argv, "main_follow_the_leader_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
    //ros::Subscriber sub = nS.subscribe<PointCloud>("points", 1, callback);
    //ros::Publisher panic_publisher = nP.advertise<std_msgs::Bool>("panic_topic", 1000);
    std::string ns = ros::this_node::getName();
    ros::ServiceClient detect_client = 
        nh.serviceClient<follow_the_leader::cmd_srv>("detect");
    
    ros::ServiceClient track_client = 
        nh.serviceClient<follow_the_leader::cmd_srv>("track");

    ros::ServiceServer resume_detection_server  = 
        nh.advertiseService("resume_detection", resume_detection_server_cb);

    std::string localization_method; 
    if(!ros::param::get("/localization_method",localization_method))  {
      ROS_FATAL_STREAM("Could not start exploration localization_method not provided");
      return -1;
    }
    if (!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s",
                (ns + "/ip_addr").c_str());
        return -1;
    }
    /* 
    if(!ros::param::get("/stats_file_addr",stats_file_addr)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return -1; 
    }
    */
    
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    control_drone(drone);
    //update_stats_file(stats_file_addr,"after control");
    follow_the_leader::cmd_srv track_srv_obj;
    follow_the_leader::cmd_srv detect_srv_obj;
    status = "resume_detection"; 
    
    int main_loop_rate = 10;
    ros::Rate loop_rate(main_loop_rate);
    while (ros::ok()) {
         /*
         detect_srv_obj.request.cmd = "start_detecting";
            if(detect_client.call(detect_srv_obj)) {
                ROS_INFO("done running detection");
            }
            else {
                ROS_ERROR("failed to call serivce for detection"); 
            }
        */

        
        if (status == "resume_detection") { 
            //calling tracking to start buffering
            track_srv_obj.request.cmd = "start_buffering";
            if(track_client.call(track_srv_obj)) {
                ROS_INFO("started buffering");
            }
            else {
                ROS_ERROR("failed to call serivce for tracking"); 
            }


            //calling detection 
            detect_srv_obj.request.cmd = "start_detecting";
            if(detect_client.call(detect_srv_obj)) {
                ROS_INFO("done running detection");
            }
            else {
                ROS_ERROR("failed to call serivce for detection"); 
            }
            
            status =  detect_srv_obj.response.status;
            if (status == "obj_detected"){
                //calling tracking to start tracking
                track_srv_obj.request.cmd = "start_tracking_for_buffered";
                track_srv_obj.request.img_id = detect_srv_obj.response.img_id;
                track_srv_obj.request.bb = detect_srv_obj.response.bb;
                if(track_client.call(track_srv_obj)) {
                    ROS_INFO("started tracking");
                }
                else {
                    ROS_ERROR("failed to call serivce for tracking"); 
                }

            }
        }
        
        //detect_client.call(detect_srv_obj);
        ros::spinOnce();
        loop_rate.sleep();
    }

}
