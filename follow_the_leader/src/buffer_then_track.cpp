#include "track.h"
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
#include <profile_manager/profiling_data_srv.h>
#include <bits/stdc++.h>
using namespace std::chrono;
int strike;
int max_strike; //= 1;
double min_allowed_tracking_treshold;
float tracking_threshold;// = .9;
typedef KCFtracker tracker_t;
tracker_t * tracker = nullptr;
bool tracker_defined;
enum State {buffer, trk_buff_imgs, waiting_for_main};
int max_n_track_before_det_count; //= 30; //number of times tracking is allowed to run before running detection again
int img_id;
std::queue<cv_bridge::CvImage> img_queue; //uesd to buffer imags while detection is running
static const std::string OPENCV_WINDOW = "Image window";
bounding_box bb;
std::queue<bounding_box> bb_queue; //uesd to buffer imags while detection is running
//std::ofstream file_to_output;
int flush_count;
int flush_count_MAX = 50;
int frame_to_process_upper_bound; //= 10;
int frame_to_process_left;
long long g_tracking_time_acc = 0;
int g_tracking_ctr = 0;
State state;
ros::Time start_hook_t(0);
ros::Time end_hook_t; 

bool tracking_cb(follow_the_leader::cmd_srv::Request &req, 
    follow_the_leader::cmd_srv::Response &res){
    if (req.cmd == "start_buffering") {
        img_queue = std::queue<cv_bridge::CvImage>(); //clear the queue
    }
    else if (req.cmd == "start_tracking_for_buffered") {
        //get the bounding box that we need to track 
        img_id = req.img_id; 
        bb.x = req.bb.x;
        bb.y = req.bb.y;
        bb.w = req.bb.w;
        bb.h = req.bb.h;
        bb.conf = req.bb.conf;
    
    }

    //moddify the state 
    if (req.cmd == "start_buffering") {
        state = buffer;
        if (start_hook_t.toSec() != 0) {  //ignore the first round
            end_hook_t = ros::Time::now(); 
            g_tracking_time_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
            g_tracking_ctr++;
        }
        else{
        }
    }else if (req.cmd == "start_tracking_for_buffered") {
        state = trk_buff_imgs;
        start_hook_t = ros::Time::now();
    }

    return true;
}

void tracking_buffered(ros::ServiceClient &resume_detection_client, ros::Publisher &bb_publisher){
    follow_the_leader::cmd_srv resume_detection_obj;
    bool sth_to_track = !img_queue.empty();
    int n_img_to_skip = (int)((float)img_queue.size()/(float)(frame_to_process_left));
    
    if (!sth_to_track){
        return;
    }
    
    if (n_img_to_skip > img_queue.size()) {
        n_img_to_skip = img_queue.size() - 1;
    }
    
    while(n_img_to_skip > 0) { //need to skip some images, other wise never be able
                               //to catch up
        if (img_queue.size() != 1){ //if the last image is popped, we can't continue 
                                  // processessing the images
            img_queue.pop();
        }
        n_img_to_skip--;
    }
    frame_to_process_left--; 
    //assert(img_queue.size() !=0);
    auto img = img_queue.front(); 
    img_queue.pop(); 
    
    if (img.header.seq >= img_id) { //only look at the images with a bigger
                                    //time stamp than the reference image 
        cv::Mat img_cpy = img.image; 

        if (!tracker_defined) {
            tracker = new tracker_t(bb, img_cpy);
            tracker_defined = true;;
        }

        bb = tracker->track(img_cpy); 
        if(img_queue.empty()){ 
            cv::Mat img_cpy_2 = img_cpy; 
            cv::rectangle(img_cpy_2, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,0,255)); //yellow
            //cv::imshow(OPENCV_WINDOW, img_cpy_2);
            cv::waitKey(2);
        }
        if (bb.conf < tracking_threshold) {
            strike++;
        }
    }
    
    if (img_queue.empty() || 
       (bb.conf < min_allowed_tracking_treshold) || 
       (strike > max_strike)) { //if tracking is done or failed, restart 
        strike = 0; 
        tracker_defined = false; 
        img_queue = std::queue<cv_bridge::CvImage>();
        frame_to_process_left = frame_to_process_upper_bound;
        resume_detection_client.call(resume_detection_obj);
        state = waiting_for_main; 
        if(!((bb.conf < min_allowed_tracking_treshold) || (strike > max_strike))) {
            follow_the_leader::bounding_box_msg bb_msg;
            bb_msg.x =  bb.x;
            bb_msg.y =  bb.y;
            bb_msg.w =  bb.w;
            bb_msg.h =  bb.h;
            bb_msg.conf =  bb.conf;
            bb_publisher.publish(bb_msg);
        }
    } 
}

void buffer_imgs_cb(const sensor_msgs::ImageConstPtr& msg) {
    if (state ==  buffer) {
        img_queue.push(*cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8));
    }else{
        ;
    }
}

void log_data_before_shutting_down(){
    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;

    profiling_data_srv_inst.request.key = "tracking_buffered_time";
    profiling_data_srv_inst.request.value = (((double)g_tracking_time_acc/g_tracking_ctr)/1e9);
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


int main(int argc, char** argv)
{
    
    tracker_defined = false; 
    //std_msgs::Bool panic_msg;
    ros::init(argc, argv, "buffer_then_track", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
    
    
    ros::Publisher bb_publisher = nh.advertise <follow_the_leader::bounding_box_msg>("/buf_img_bb_topic", 4);
    ros::Subscriber raw_image_sub  = nh.subscribe("/Airsim/right/image_raw", 1, buffer_imgs_cb);
    ros::ServiceServer track_server =  nh.advertiseService("track", tracking_cb);

    //file_to_output.open("/home/nvidia/catkin_ws/src/mav-bench/follow_the_leader/src/tracking_output.txt");

    if(!ros::param::get("/buffer_then_track/max_n_track_before_det_count",max_n_track_before_det_count))  {
      ROS_FATAL_STREAM("Could not start buffer_then_track cause max_n_track_before_det_count not provided");
      return -1;
    }
    
    
    
    if (!ros::param::get("/buffer_then_track/tracking_threshold", tracking_threshold)) {
        ROS_FATAL("Could not start tracing_node cause tracking_threshol dmissing! Looking for");
        return -1;
    }
    if(!ros::param::get("/buffer_then_track/max_strike",max_strike))  {
      ROS_FATAL_STREAM("Could not start buffer_then_track cause max_strike not provided");
      return -1;
    }
    if(!ros::param::get("/buffer_then_track/min_allowed_tracking_treshold",min_allowed_tracking_treshold))  {
      ROS_FATAL_STREAM("Could not start buffer_then_track cause min_allowed_tracking_treshold not provided");
      return -1;
    }

     if(!ros::param::get("/frame_to_process_upper_bound",frame_to_process_upper_bound))  {
      ROS_FATAL_STREAM("Could not start buffer_then_track cause frame_to_process_upper_bound not provided");
      return -1;
    }
    ros::ServiceClient resume_detection_client = 
        nh.serviceClient<follow_the_leader::cmd_srv>("resume_detection");

    follow_the_leader::cmd_srv track_srv_obj;

    frame_to_process_left = frame_to_process_upper_bound;
    
    while (ros::ok()) {
        if (state == trk_buff_imgs) { 
            tracking_buffered(resume_detection_client, bb_publisher); 
        }
        ros::spinOnce();
    }
}
