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

int strike;
int max_strike = 5;
int tracking_threshold = .7;
typedef KCFtracker tracker_t;
tracker_t * tracker = nullptr;
bool tracker_defined;
std::string status;
int tracking_count; //number of times tracking has run after detection
int max_tracking_count = 30; //number of times tracking is allowed to run before running detection again
int img_id;
std::queue<cv_bridge::CvImage> img_queue; //uesd to buffer imags while detection is running

static const std::string OPENCV_WINDOW = "Image window";
bounding_box bb;

std::queue<bounding_box> bb_queue; //uesd to buffer imags while detection is running

bool tracking_cb(follow_the_leader::cmd_srv::Request &req, 
    follow_the_leader::cmd_srv::Response &res){
    if (req.cmd == "start_buffering") {
        img_queue = std::queue<cv_bridge::CvImage>();
        tracking_count = 0; 
        ROS_INFO_STREAM("clearning buffers");
    
    }
    else if (req.cmd == "start_tracking") {
        ROS_INFO_STREAM("starting to track");
        //ROS_INFO_STREAM("boundign box of the detection"<<req.bb.x<<" " <<req.bb.y<< " " << req.bb.w << " " <<req.bb.h);
        img_id = req.img_id; 
        //bb = req.bb; 
         
        bb.x = req.bb.x;
        bb.y = req.bb.y;
        bb.w = req.bb.w;
        bb.h = req.bb.h;
        bb.conf = req.bb.conf;
        
    }

    status = req.cmd;
    return true;
}

bounding_box tracking(ros::ServiceClient &resume_detection_client){
    tracking_count++; 
    //ROS_INFO_STREAM("here is the count"<<count); 
    follow_the_leader::cmd_srv resume_detection_obj;
    //for(auto img:  img_queue) {
    //ROS_INFO_STREAM("buffer size"<<img_queue.size());
    while (!img_queue.empty())  {
       auto img = img_queue.front(); 
       img_queue.pop(); 
       if (img.header.seq >= img_id) {
           //if (img.header.seq == img_id) {
            //ROS_INFO_STREAM("found equal");
           //}
           if (!tracker_defined) {
               //ROS_INFO_STREAM("boundign box of the tracking instantiation"<<bb.x<<" " <<bb.y<< " " << bb.w << " " <<bb.h);
               tracker = new tracker_t(bb, img.image);
               tracker_defined = true;;
           }
           bb = tracker->track(img.image); 
           cv::Mat img_to_show;
           cv::Mat img_cpy = img.image; 
           cv::rectangle(img_cpy, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(0,255,255)); //yellow
           //cv::Size size(512, 512);
           //resize(img.image, img_to_show, size);
           cv::imshow(OPENCV_WINDOW, img.image);
           cv::waitKey(3);
           //cv::waitKey(500);
           //cv::destroyAllWindows();
           
       }
   }
    
    //only the last bb is push. This is b/c we only buffer the images 
    //so the tracker won't lose it, however, the most recent bb should 
    // be passed to the pid so we can respond according to the latest bb(where
    // the target is at the moement)
    if (bb.conf < tracking_threshold) {
        ROS_INFO_STREAM("strike is "<<strike); 
        strike++;

    }
    else {
        ROS_INFO_STREAM("bb.h"<<bb.h); 
        bb_queue.push(bb);
    }

    if (strike > max_strike || tracking_count > max_tracking_count) { //termination of tracker
                              //reseting all the data structures, varaibles
       //delete tracker; 
       tracking_count = 0; 
       strike = 0; 
       tracker_defined = false; 
       img_queue = std::queue<cv_bridge::CvImage>();
       resume_detection_client.call(resume_detection_obj);
       status = "waiting_for_main"; 
       cv::destroyAllWindows();
   }
}


void sample_images_cb(const sensor_msgs::ImageConstPtr& msg) {
    if (status ==  "start_buffering" || status =="start_tracking") {
        img_queue.push(*cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8));
    }
}

int main(int argc, char** argv)
{
    
    tracker_defined = false; 
    //std_msgs::Bool panic_msg;
    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh;
    
    
    ros::Publisher bb_publisher = nh.advertise <follow_the_leader::bounding_box_msg>("/bb_topic", 4);
    ros::Subscriber raw_image_sub  = nh.subscribe("/Airsim/right/image_raw", 1, sample_images_cb);
    ros::ServiceServer track_server = 
        nh.advertiseService("track", tracking_cb);

    ros::ServiceClient resume_detection_client = 
        nh.serviceClient<follow_the_leader::cmd_srv>("resume_detection");

    follow_the_leader::cmd_srv track_srv_obj;

    int tracking_loop_rate = 10;
    ros::Rate loop_rate(tracking_loop_rate);
    while (ros::ok()) {
        if (status == "start_tracking") { 
            tracking(resume_detection_client); 
            while(!bb_queue.empty()) {
                auto bb = bb_queue.front(); 
                follow_the_leader::bounding_box_msg bb_msg;
                bb_msg.x =  bb.x;
                bb_msg.y =  bb.y;
                bb_msg.w =  bb.w;
                bb_msg.h =  bb.h;
                bb_msg.conf =  bb.conf;
                ROS_INFO_STREAM("bb_msg"<<bb_msg.h); 
                bb_publisher.publish(bb_msg);
                bb_queue.pop();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }            
}
