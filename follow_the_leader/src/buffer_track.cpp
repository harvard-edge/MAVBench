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
bool tracker_defined;
std::string status;
int tracking_count; //number of times tracking has run after detection
int max_n_track_before_det_count; //= 30; //number of times tracking is allowed to run before running detection again
int img_id;
std::queue<cv_bridge::CvImage> img_queue; //uesd to buffer imags while detection is running

static const std::string OPENCV_WINDOW = "Image window";
bounding_box bb;

std::queue<bounding_box> bb_queue; //uesd to buffer imags while detection is running
std::ofstream file_to_output;
int flush_count;
int flush_count_MAX = 50;
int FRAME_TO_PROCESS_UPPER_BOUND = 10;
int frame_to_process_left;
bool tracking_cb(follow_the_leader::cmd_srv::Request &req, 
    follow_the_leader::cmd_srv::Response &res){
    if (req.cmd == "start_buffering") {
        img_queue = std::queue<cv_bridge::CvImage>();
        tracking_count = 0; 
        //ROS_INFO_STREAM("clearning buffers");
    }
    else if (req.cmd == "start_tracking_for_buffered") {
        //ROS_INFO_STREAM("starting to track");
        img_id = req.img_id; 
         
        bb.x = req.bb.x;
        bb.y = req.bb.y;
        bb.w = req.bb.w;
        bb.h = req.bb.h;
        bb.conf = req.bb.conf;
        
    }

    status = req.cmd;
    return true;
}

bounding_box tracking_buffered(ros::ServiceClient &resume_detection_client, ros::Publisher &bb_publisher){
    
    //ROS_INFO_STREAM("start tracking buffered");
    follow_the_leader::cmd_srv resume_detection_obj;
    
    //DEBUGING vars
    tracking_count++; 
    steady_clock::time_point fnc_t_s; //total function time s
    steady_clock::time_point fnc_t_e; //total function time e
    steady_clock::time_point trk_s; //one invocation of tracker start
    steady_clock::time_point trk_e; //one invocation of tracker end

    fnc_t_s  = steady_clock::now();
    int trk_counter = 0; //number of of tracking invocation 
    int trk_total = 0; //total time spent on tracker
    int queue_size; 
    bool about_to_exit = false;    
    //DEBUGGING: only set up debugging vars if queue not empy 
    bool sth_to_track = !img_queue.empty();
    if (sth_to_track){
        queue_size = img_queue.size(); 
        trk_counter = 0;
    }
    
    
    //while (!img_queue.empty())  {
     int n_img_to_skip = (int)((float)img_queue.size()/(float)(frame_to_process_left));
     //ROS_INFO_STREAM("queuesize "<< img_queue.size() << "  ftpl:"<<frame_to_process_left<<" img_to_skip:"<<n_img_to_skip);
     if (n_img_to_skip > img_queue.size()) {
           n_img_to_skip = img_queue.size() - 1;
       }
       while(n_img_to_skip > 0) {
            img_queue.pop();
            n_img_to_skip--;
       }
       frame_to_process_left--; 
       
       auto img = img_queue.front(); 
       img_queue.pop(); 
       if (img.header.seq >= img_id) {
           cv::Mat img_cpy = img.image; 
           cv::Size size(512, 288);
           //--- inflating the img 
           cv::Mat img_inflated; //required since detection has a lower limit on the size 
           resize(img_cpy, img_inflated, size);

           if (!tracker_defined) {
               tracker = new tracker_t(bb, img_inflated);
               draw_now = true; 
               tracker_defined = true;;
           }
           
           //bb = tracker->track(img.image); 
           trk_s  = steady_clock::now();
           bb = tracker->track(img_inflated); 
           trk_e  = steady_clock::now();
           auto trk__t = duration_cast<milliseconds>(trk_e - trk_s).count();
           trk_total += trk__t; 
           trk_counter++;

           //cv::Mat img_to_show;
           //cv::Mat img_cpy = img.image; 
           if(img_queue.empty()){ 
               cv::Mat img_cpy_2 = img_inflated; 
               cv::rectangle(img_cpy_2, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,0,255)); //yellow
               cv::imshow(OPENCV_WINDOW, img_cpy_2);
               //cv::imshow(OPENCV_WINDOW, img_cpy);
               cv::waitKey(10);
           }
           if (bb.conf < tracking_threshold) {
               //ROS_INFO_STREAM("conf is lowerthan threshold "<<bb.conf); 
               strike++;
            }

           //break out if any of the following true
           //if((bb.conf < min_allowed_tracking_treshold) || (strike > max_strike)) {
            //    break; 
           //}
       }
   //}

    if (img_queue.empty() || (bb.conf < min_allowed_tracking_treshold) || (strike > max_strike)) {
        ROS_INFO_STREAM("img_queue is empty");
        tracking_count = 0; 
        strike = 0; 
        tracker_defined = false; 
        img_queue = std::queue<cv_bridge::CvImage>();
        resume_detection_client.call(resume_detection_obj);
        status = "waiting_for_main"; 
        frame_to_process_left = FRAME_TO_PROCESS_UPPER_BOUND;
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
     /* 
    //only the last bb is push. This is b/c we only buffer the images 
    //so the tracker won't lose it, however, the most recent bb should 
    // be passed to the pid so we can respond according to the latest bb(where
    // the target is at the moement)
    if (bb.conf < tracking_threshold) {
        //ROS_INFO_STREAM("conf is lowerthan threshold "<<bb.conf); 
        strike++;

    }else {
        bb_queue.push(bb);
    }
    */
    
    /*
    if (strike > max_strike || tracking_count > max_n_track_before_det_count || (bb.conf < min_allowed_tracking_treshold)) { //termination of tracker
       if (strike > max_strike) {
         //ROS_INFO_STREAM("max-Strike" <<strike); 
       }
    if (tracking_count > max_n_track_before_det_count){
         //ROS_INFO_STREAM("tracking count high" <<tracking_count); 
       }
    if (bb.conf < min_allowed_tracking_treshold){
         //ROS_INFO_STREAM("bb.conf is too low" <<bb.conf); 
       }

       tracking_count = 0; 
       strike = 0; 
       tracker_defined = false; 
       img_queue = std::queue<cv_bridge::CvImage>();
       resume_detection_client.call(resume_detection_obj);
       status = "waiting_for_main"; 
       //cv::destroyAllWindows();
       
       about_to_exit = true; 
       file_to_output<<"exiting tracker"<<std::endl;
    
      //bb_msg.w =  bb.w;
      //bb_msg.h =  bb.h;
      //bb_msg.conf =  bb.conf;

    }

    if(sth_to_track) {
        steady_clock::time_point fnc_t_e = steady_clock::now();
        auto fnc_t = duration_cast<milliseconds>(fnc_t_e - fnc_t_s).count();
        auto avg_trk_t = (float)trk_total/(float)trk_counter;
        if (queue_size > 1 || about_to_exit){ 
            file_to_output<<"Q size:"<<queue_size<<"  func t:"<<fnc_t<<"ms"<< "  avg trk t:"<< avg_trk_t<<"ms"<<std::endl;
        }
    
    
    }
    */   


    //ROS_INFO_STREAM("done buffering ");
}


void sample_images_cb(const sensor_msgs::ImageConstPtr& msg) {
    if (status ==  "start_buffering") {
        img_queue.push(*cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8));
    }
   static int count = 0;
   count++; 
    if (count % 40 == 0){
       //ROS_INFO_STREAM("img count is "<< count);
   }
}

int main(int argc, char** argv)
{
    
    tracker_defined = false; 
    //std_msgs::Bool panic_msg;
    ros::init(argc, argv, "buffer_track_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
    
    ros::Publisher bb_publisher = nh.advertise <follow_the_leader::bounding_box_msg>("/buf_img_bb_topic", 4);
    ros::Subscriber raw_image_sub  = nh.subscribe("/Airsim/right/image_raw", 1, sample_images_cb);
    ros::ServiceServer track_server = 
        nh.advertiseService("track", tracking_cb);


    file_to_output.open("/home/nvidia/catkin_ws/src/mav-bench/follow_the_leader/src/tracking_output.txt");

    if(!ros::param::get("/buffer_track_node/max_n_track_before_det_count",max_n_track_before_det_count))  {
      ROS_FATAL_STREAM("Could not start buffer_track_node cause max_n_track_before_det_count not provided");
      return -1;
    }
    if (!ros::param::get("/buffer_track_node/tracking_threshold", tracking_threshold)) {
        ROS_FATAL("Could not start tracing_node cause tracking_threshol dmissing! Looking for");
        return -1;
    }
    if(!ros::param::get("/buffer_track_node/max_strike",max_strike))  {
      ROS_FATAL_STREAM("Could not start buffer_track_node cause max_strike not provided");
      return -1;
    }
    if(!ros::param::get("/buffer_track_node/min_allowed_tracking_treshold",min_allowed_tracking_treshold))  {
      ROS_FATAL_STREAM("Could not start buffer_track_node cause min_allowed_tracking_treshold not provided");
      return -1;
    }

    ros::ServiceClient resume_detection_client = 
        nh.serviceClient<follow_the_leader::cmd_srv>("resume_detection");

    follow_the_leader::cmd_srv track_srv_obj;

    int tracking_loop_rate = 50;
    ros::Rate loop_rate(tracking_loop_rate);
    frame_to_process_left = FRAME_TO_PROCESS_UPPER_BOUND;
    while (ros::ok()) {
        if (status == "start_tracking_for_buffered") { 
            tracking_buffered(resume_detection_client, bb_publisher); 
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
