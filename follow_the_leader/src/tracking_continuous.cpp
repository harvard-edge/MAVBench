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
#include <bits/stdc++.h>
using namespace std::chrono;
bool draw_now;
int strike;
int max_strike; //= 1;
double min_allowed_tracking_treshold;
float tracking_threshold;// = .9;
typedef KCFtracker tracker_t;
tracker_t * tracker = nullptr;
bool tracker_defined = false;
int max_n_track_before_det_count; //= 30; //number of times tracking is allowed to run before running detection again
int img_id;
std::queue<cv_bridge::CvImage> img_queue; //uesd to buffer imags while detection is running
bool first_ever_bb_received = false;
bounding_box buf_img_bb;

static const std::string OPENCV_WINDOW = "Image window";
bounding_box bb;

std::queue<bounding_box> bb_queue; //uesd to buffer imags while detection is running
//std::ofstream file_to_output;
int flush_count;
int flush_count_MAX = 50;

void tracking(ros::Publisher &bb_publisher){
    
    follow_the_leader::cmd_srv resume_detection_obj;
    int queue_size; 
    bool sth_to_track = !img_queue.empty();
    if (sth_to_track){
        queue_size = img_queue.size(); 
    }
    
    if(img_queue.empty()){
        return;
    }
    auto& img = img_queue.front(); 
    cv::Mat img_cpy = img.image; 

    if (!tracker_defined) {
        tracker = new tracker_t(buf_img_bb, img_cpy);
        tracker_defined = true;
    }

    bb = tracker->track(img_cpy); 
    
    cv::Mat img_cpy_2 = img_cpy; 
    cv::rectangle(img_cpy_2, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,255,0)); //yellow
    cv::imshow(OPENCV_WINDOW, img_cpy_2);
    cv::waitKey(2);
    
    follow_the_leader::bounding_box_msg bb_msg;
    bb_msg.x =  bb.x;
    bb_msg.y =  bb.y;
    bb_msg.w =  bb.w;
    bb_msg.h =  bb.h;
    bb_msg.conf =  bb.conf;
    bb_publisher.publish(bb_msg);
    /* 
    if (bb.conf < tracking_threshold) {
        strike++;
    }
    */
}


void buffer_imgs_cb(const sensor_msgs::ImageConstPtr& msg) {
   img_queue = std::queue<cv_bridge::CvImage>();
   img_queue.push(*cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8));
}

void buffer_img_bb_cb(const follow_the_leader::bounding_box_msg::ConstPtr& msg){
    first_ever_bb_received = true; 
    buf_img_bb.x = msg->x;
    buf_img_bb.y = msg->y;
    buf_img_bb.w = msg->w;
    buf_img_bb.h = msg->h;
    buf_img_bb.conf  = msg->conf;
    tracker_defined = false; 
}

int main(int argc, char** argv)
{
    tracker_defined = false; 
    ros::init(argc, argv, "tracking_continuous", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
    
    ros::Publisher bb_publisher = nh.advertise <follow_the_leader::bounding_box_msg>("/bb_topic", 4);
    ros::Subscriber raw_image_sub  = nh.subscribe("/Airsim/right/image_raw", 1, buffer_imgs_cb);
    ros::Subscriber buf_img_sub = nh.subscribe("/buf_img_bb_topic", 1, buffer_img_bb_cb);

    //file_to_output.open("/home/nvidia/catkin_ws/src/mav-bench/follow_the_leader/src/tracking_output.txt");

    if(!ros::param::get("/tracking_continuous/max_n_track_before_det_count",max_n_track_before_det_count))  {
      ROS_FATAL_STREAM("Could not start tracking_continuous cause max_n_track_before_det_count not provided");
      return -1;
    }
    if (!ros::param::get("/tracking_continuous/tracking_threshold", tracking_threshold)) {
        ROS_FATAL("Could not start tracing_node cause tracking_threshol dmissing! Looking for");
        return -1;
    }
    if(!ros::param::get("/tracking_continuous/max_strike",max_strike))  {
      ROS_FATAL_STREAM("Could not start tracking_continuous cause max_strike not provided");
      return -1;
    }
    if(!ros::param::get("/tracking_continuous/min_allowed_tracking_treshold",min_allowed_tracking_treshold))  {
      ROS_FATAL_STREAM("Could not start tracking_continuous cause min_allowed_tracking_treshold not provided");
      return -1;
    }

    follow_the_leader::cmd_srv track_srv_obj;
    while (ros::ok()) {
        if (first_ever_bb_received) 
        {
            tracking(bb_publisher); 
        }
        ros::spinOnce();
    }            
}

