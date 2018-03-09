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
#include "error.h"
#include "bounding_box.h"
using namespace std;

std::string state;
std::string ns;
std::string ip_addr__global;
long long g_obj_detection_time_including_ros_over_head_acc = 0;
long long g_tracking_time_acc = 0;
int g_obj_detection_ctr = 0;
int g_tracking_ctr = 0;
std::string g_mission_status = "completed";
long long g_error_accumulate = 0;
int g_error_ctr = 0;
int g_tracked_time=0;
ros::Time following_start_t; 
ros::Time following_end_t; 

int image_w__global;// = 400;
int  image_h__global; //= 400; //this must be equal to the img being pulled in from airsim
float height_ratio;
std::string g_localization_method; 
int g_detec_fail_ctr_threshold; 
std::string g_supervisor_mailbox;


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

    profiling_data_srv_inst.request.key = "following_time";
    profiling_data_srv_inst.request.value = (following_end_t - following_start_t).toSec();
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "mission_status";
    profiling_data_srv_inst.request.value = (g_mission_status == "completed" ? 1.0: 0.0);
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "error";
    profiling_data_srv_inst.request.value = ((double)g_error_accumulate/g_error_ctr)/1000;
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

int initialize_parameters(){
    if(!ros::param::get("/localization_method",g_localization_method))  {

      ROS_FATAL_STREAM("Could not start exploration localization_method not provided");
      return -1;
    }
    if(!ros::param::get("/detec_fail_ctr_threshold",g_detec_fail_ctr_threshold))  {
      ROS_FATAL_STREAM("Could not start follow the leader detec_fail_ctr_threshold not provided");
      return -1;
    }
    
    if (!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s",
                (ns + "/ip_addr").c_str());
        return -1;
    }
    
    
    if(!ros::param::get("/image_w__global",image_w__global)||
            !ros::param::get("/image_h__global",image_h__global)||
            !ros::param::get("/height_ratio",height_ratio)){
          
        ROS_ERROR_STREAM("Could not start follow the leader cause one of the image related parameters are missing");
           return -1; 
    }
    
    if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
      ROS_FATAL_STREAM("Could not start mapping supervisor_mailbox not provided");
      return -1;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_the_leader_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
    std::string ns = ros::this_node::getName();
    initialize_parameters(); 
    ros::ServiceClient detect_client = 
        nh.serviceClient<follow_the_leader::cmd_srv>("detect");
    
    ros::ServiceClient track_client = 
        nh.serviceClient<follow_the_leader::cmd_srv>("track");

    ros::ServiceClient shutdown_client = 
        nh.serviceClient<follow_the_leader::shut_down>("shutdown_topic");

    ros::ServiceServer resume_detection_server  = 
        nh.advertiseService("resume_detection", resume_detection_server_cb);

    ros::Time start_hook_t, end_hook_t;  
    int detec_fail_ctr = 0; 
    bool call_service_succesfull; 
    uint16_t port = 41451;
    follow_the_leader::cmd_srv cmd_srv_inst;
    state = "resume_detection"; 
    
    Drone drone(ip_addr__global.c_str(), port, g_localization_method);
    control_drone(drone);

    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profiling_data_srv_inst.request.key = "start_profiling";
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    following_start_t =  ros::Time::now(); 

    while (ros::ok() && (detec_fail_ctr < g_detec_fail_ctr_threshold)) {
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
            
            bounding_box bb;
            bb.x = cmd_srv_inst.response.bb.x;
            bb.y = cmd_srv_inst.response.bb.y;
            bb.w = cmd_srv_inst.response.bb.w;
            bb.h = cmd_srv_inst.response.bb.h;
            bb.conf  = cmd_srv_inst.response.bb.conf;
            error error_inst(bb, image_h__global, 
                    image_w__global, height_ratio);
            g_error_accumulate +=  (error_inst.full)*1000;
            g_error_ctr +=1;
            
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

    if (detec_fail_ctr >= g_detec_fail_ctr_threshold) {
        g_mission_status = "failed";
        following_end_t =  ros::Time::now(); 
    
    }
    log_data_before_shutting_down();
    signal_supervisor(g_supervisor_mailbox, "kill"); 
    ros::shutdown();
}
