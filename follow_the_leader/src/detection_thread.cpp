#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "follow_the_leader/bounding_box_msg.h"
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

#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>
#include "common.h"
#include <cstring>
#include <string>
#include "bounding_box.h"
#include "follow_the_leader/cmd_srv.h"
#include "objdetect.h"


typedef YOLODetector detector_t;
static const std::string OPENCV_WINDOW = "Image window";

cv_bridge::CvImage cv_img;
int img_id;
detector_t detector;

void sample_images_cb(const sensor_msgs::ImageConstPtr& msg) {
      cv_img = *cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      img_id = cv_img.header.seq;

}

bool detection_cb(follow_the_leader::cmd_srv::Request &req, 
    follow_the_leader::cmd_srv::Response &res)
{
    const double detect_thresh = 0.8;
    //cv::Mat img;
    cv::Mat depth;
    bounding_box bb = {-3, -3, -3, -3, -3};
    //img = drone.read_frame();
    
    cv::Mat img_cpy = cv_img.image; 
    /*
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      exit(0);
    } 
   */
    //ROS_INFO_STREAM("header"<<cv_ptr->header.seq);
    //return; 
    //ROS_INFO_STREAM("before detection");	
        //ROS_INFO_STREAM("blah blah");
    bb = detector.detect_person(img_cpy);
    //ROS_INFO_STREAM("after detection");	
    //mapping_and_SAR::OD result; 
    

    if(bb.conf >= detect_thresh) {
        ROS_INFO_STREAM("found the object"<< bb.conf);
        cv::Mat img_to_show;
        cv::Mat img_cpy = cv_img.image; 
        cv::rectangle(img_cpy, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,255,0)); //yellow
        //cv::Size size(512, 512);
        //resize(img_cpy, img_to_show, size);
        //system(("rosnode kill " + mav_name__global + "/SAR").c_str());
        cv::imshow(OPENCV_WINDOW, cv_img.image);
        cv::waitKey(1000);

        res.status= "obj_detected";
        res.bb.x = bb.x;
        res.bb.y = bb.y;
        res.bb.w = bb.w;
        res.bb.h = bb.h;
        res.bb.conf = bb.conf;
        res.img_id = img_id;
       cv::destroyAllWindows();
    }else {
        res.status = "resume_detection"; 
    }
    return true;
}           


/*
bool detection_cb(follow_the_leader::cmd_srv::Request &req, 
    follow_the_leader::cmd_srv::Response &res){
    std::string cmd("");
    std::cin >> cmd;
    
    // showing the img 
    cv::Mat img_to_show;
    cv::Mat img_cpy = cv_img.image; 
    //cv::rectangle(img_cpy, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(0,255,255)); //yellow
    cv::Size size(512, 512);
    resize(img_cpy, img_to_show, size);
    //for (int i = 0; i < 5; i++){ 
        cv::imshow(OPENCV_WINDOW, img_to_show);
        cv::waitKey(2000);
        cv::destroyAllWindows();
    //}

    //ROS_INFO_STREAM("inside detection_cb" <<  cmd); 
    res.status= cmd;
    res.img_id = img_id;
    return true;
}
*/
int main(int argc, char** argv)
{
    //std_msgs::Bool panic_msg;
    ros::init(argc, argv, "detection_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
    uint16_t port = 41451;
    std::string ip_addr__global;
    if (!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL_STREAM("Could not start exploration. Parameter missing! Looking for /ip_addr");
        return -1;
    }

    ros::Subscriber raw_image_sub  = nh.subscribe("/Airsim/right/image_raw", 1, sample_images_cb);
    ros::ServiceServer detect_server = 
        nh.advertiseService("detect", detection_cb);

    //ros::ServiceClient img_id_client = 
     //   nh.serviceClient<follow_the_leader::img_id>("img_id");



    int detection_loop_rate = 10;
    ros::Rate loop_rate(detection_loop_rate);

    while(ros::ok) {
         
        ros::spinOnce();
    }
    /* 
    while (ros::ok()) {
        
     
        if (status != "tracking") { 
            //calling tracking to start buffering
            track_srv_obj.request = "start_buffering";
            if(track_client.call(track_srv_obj)) {
                ROS_INFO("started buffering on the tracking node");
            }
            else {
                ROS_ERROR("failed to call serivce for tracking"); 
            }


            //calling detection 
            detect_srv_obj.request = "start_detecting";
            if(detect_client.call(detect_srv_obj)) {
                ROS_INFO("started detecting objects");
            }
            else {
                ROS_ERROR("failed to call serivce for detection"); 
            }
            
            status =  detect_srv_obj.response.status;
            if (status == "obj_found"){
                //calling tracking to start tracking
                track_srv_obj.request = "start_tracking";
                if(track_client.call(track_srv_obj)) {
                    ROS_INFO("started tracking on the tracking node");
                }
                else {
                    ROS_ERROR("failed to call serivce for tracking"); 
                }
            }
        }
    }

    */
}
