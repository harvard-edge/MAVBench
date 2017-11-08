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


static const std::string OPENCV_WINDOW = "Image window";

cv_bridge::CvImage cv_img;
int img_id;

void sample_images_cb(const sensor_msgs::ImageConstPtr& msg) {
    cv_img = *cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      img_id = cv_img.header.seq;
      cv::imshow(OPENCV_WINDOW, cv_img.image);
      cv::waitKey(3);

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
    ros::init(argc, argv, "detection_node");
    ros::NodeHandle nh;
    
    uint16_t port = 41451;
    std::string ip_addr__global;
    if (!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL_STREAM("Could not start exploration. Parameter missing! Looking for /ip_addr");
        return -1;
    }

    ros::Subscriber raw_image_sub  = nh.subscribe("/Airsim/right/image_raw", 1, sample_images_cb);
    //ros::ServiceServer detect_server = 
    //    nh.advertiseService("detect", detection_cb);

    int detection_loop_rate = 10;
    ros::Rate loop_rate(detection_loop_rate);

    while(ros::ok) {
         
        ros::spinOnce();
    }
}
