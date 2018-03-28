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
#include <profile_manager/profiling_data_srv.h>
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


//--- profiling related variables
ros::Time start_hook_t; 
ros::Time end_hook_t; 
int g_obj_detection_ctr = 0;
long long g_obj_detection_time_acc=0;

void log_data_before_shutting_down(){
    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;

    profiling_data_srv_inst.request.key = "obj_detection_kernel";
    profiling_data_srv_inst.request.value = ((double)g_obj_detection_time_acc/g_obj_detection_ctr)/1e9;
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
     
    start_hook_t =  ros::Time::now(); 
    bb = detector.detect_person(img_cpy);
    end_hook_t =  ros::Time::now(); 
    g_obj_detection_ctr++;
    g_obj_detection_time_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
    //ROS_INFO_STREAM("detection took"<<(end_hook_t - start_hook_t).toSec());

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
    signal(SIGINT, sigIntHandlerPrivate);
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
