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
#include "common.h"
//#include "controllers/DroneControllerBase.hpp"
//#include "common/Common.hpp"
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
#include <fstream>
using namespace std;
using namespace chrono;
ofstream file_to_output;
//std::string stats_file_addr;
typedef YOLODetector detector_t;
static const std::string OPENCV_WINDOW = "Image window";
double detect_thresh;// = 0.8;
cv_bridge::CvImage cv_img;
int img_id;
detector_t detector;

void buufer_imgs_cb(const sensor_msgs::ImageConstPtr& msg) {
      cv_img = *cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      img_id = cv_img.header.seq;
}

bool detection_cb(follow_the_leader::cmd_srv::Request &req, 
    follow_the_leader::cmd_srv::Response &res)
{
    cv::Mat depth;
    bounding_box bb = {-3, -3, -3, -3, -3};
    cv::Mat img_cpy = cv_img.image; 
    bb = detector.detect_person(img_cpy);
    
    cv::Mat img_cpy_2 = img_cpy; 
    cv::rectangle(img_cpy_2, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,255,0)); //yellow
    
    if(bb.conf >= detect_thresh) {
       //cv::imshow(OPENCV_WINDOW, img_cpy_2);
       //cv::waitKey(40);
        res.status= "obj_detected";
        res.bb.x = bb.x;
        res.bb.y = bb.y;
        res.bb.w = bb.w;
        res.bb.h = bb.h;
        res.bb.conf = bb.conf;
        res.img_id = img_id;
       
    }else {
        res.status = "resume_detection"; 
    }
    return true;
}           

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
    uint16_t port = 41451;
    std::string ip_addr__global;
    if (!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL_STREAM("Could not start exploration. Parameter missing! Looking for /ip_addr");
        return -1;
    }

    if (!ros::param::get("/detect_thresh", detect_thresh)) {
        ROS_FATAL_STREAM("Could not start detection. Parameter missing! Looking for /detect_thresh");
        return -1;
    }

    ros::Subscriber raw_image_sub  = nh.subscribe("/Airsim/right/image_raw", 1, buufer_imgs_cb);
    ros::ServiceServer detect_server = 
        nh.advertiseService("detect", detection_cb);

    while(ros::ok) {
        ros::spinOnce();
    }
}
