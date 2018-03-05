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
//#include "configs.h"
//#include "misc.h"
//#include "log__class.h"
#include "follow_the_leader/cmd_srv.h"
#include "common.h"
#include "follow_the_leader/shut_down.h"
#include <profile_manager/profiling_data_srv.h>
using namespace std;

std::string state;
std::string ns;
std::string ip_addr__global;
long long g_obj_detection_time_including_ros_over_head_acc = 0;
long long g_tracking_time_acc = 0;
int g_obj_detection_ctr = 0;
int g_tracking_ctr = 0;

void log_data_before_shutting_down(){
    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;

    profiling_data_srv_inst.request.key = "object_detection_time";
    profiling_data_srv_inst.request.value = ((double)g_obj_detection_time_including_ros_over_head_acc/g_obj_detection_ctr)/1e9;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
}

void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        ros::shutdown();
    }
    exit(0);
}

bool resume_detection_server_cb(follow_the_leader::cmd_srv::Request &req, 
    follow_the_leader::cmd_srv::Response &res){
    state = "resume_detection";
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_the_leader_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
    std::string ns = ros::this_node::getName();
    ros::ServiceClient detect_client = 
        nh.serviceClient<follow_the_leader::cmd_srv>("detect");
    
    ros::ServiceClient track_client = 
        nh.serviceClient<follow_the_leader::cmd_srv>("track");

    ros::ServiceClient shutdown_client = 
        nh.serviceClient<follow_the_leader::shut_down>("shutdown_topic");

    ros::ServiceServer resume_detection_server  = 
        nh.advertiseService("resume_detection", resume_detection_server_cb);

    std::string localization_method; 
    int detec_fail_ctr_threshold; 
    ros::Time start_hook_t, end_hook_t;  
    int detec_fail_ctr = 0; 
    bool call_service_succesfull; 
    uint16_t port = 41451;
    follow_the_leader::cmd_srv cmd_srv_inst;
    state = "resume_detection"; 
    
    if(!ros::param::get("/localization_method",localization_method))  {
      ROS_FATAL_STREAM("Could not start exploration localization_method not provided");
      return -1;
    }
    if(!ros::param::get("/detec_fail_ctr_threshold",detec_fail_ctr_threshold))  {
      ROS_FATAL_STREAM("Could not start follow the leader detec_fail_ctr_threshold not provided");
      return -1;
    }
    
    if (!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s",
                (ns + "/ip_addr").c_str());
        return -1;
    }
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    control_drone(drone);

    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profiling_data_srv_inst.request.key = "start_profiling";
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    while (ros::ok() && (detec_fail_ctr < detec_fail_ctr_threshold)) {
        if (state != "resume_detection") {
            ros::spinOnce();
            continue;
        }
        
        //call buff_track to buffer data 
        cmd_srv_inst.request.cmd = "start_buffering";
        if(!track_client.call(cmd_srv_inst)){
            ROS_ERROR("failed to call serivce for tracking"); 
        }

        //call detection to detect
        cmd_srv_inst.request.cmd = "start_detecting";
        start_hook_t = ros::Time::now();
        call_service_succesfull =  detect_client.call(cmd_srv_inst);
        end_hook_t = ros::Time::now(); 
        g_obj_detection_time_including_ros_over_head_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
        g_obj_detection_ctr++; 
        if(!call_service_succesfull) {
            ROS_ERROR("failed to call serivce for detection"); 
        }
        state = cmd_srv_inst.response.status ; 

        //if detected, call buff_track to track  
        if (state == "obj_detected"){
            cmd_srv_inst.request.cmd = "start_tracking_for_buffered";
            cmd_srv_inst.request.img_id = cmd_srv_inst.response.img_id;
            cmd_srv_inst.request.bb = cmd_srv_inst.response.bb;
            call_service_succesfull = track_client.call(cmd_srv_inst);
            if(!call_service_succesfull) {
                ROS_ERROR("failed to call serivce for tracking"); 
            }
            detec_fail_ctr = 0; 
        }else{
            detec_fail_ctr++;
        }
        ros::spinOnce();
    }

    log_data_before_shutting_down();
    ros::shutdown();
}
