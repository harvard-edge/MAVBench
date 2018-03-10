#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>
#include <limits>
#include <stdint.h>
#include <math.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Drone.h"
#include "objdetect.h"
#include "string"
#include "common.h"
#include "signal.h"

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <thread>
#include <chrono>
#include "coord.h"
#include <ros/ros.h>
#include <ros/package.h>
//#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <multiagent_collision_check/Segment.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nbvplanner/nbvp_srv.h>
#include <mapping_and_sar/OD.h>
#include "common/Common.hpp"
#include <fstream>
#include "Drone.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <profile_manager/profiling_data_srv.h>

static const std::string OPENCV_WINDOW = "Image window";
static std::string mav_name__global;
YOLODetector detector;
std::string stats_file_addr;
static mapping_and_sar::OD result;

//profiling variable
ros::Time start_hook_t; 
ros::Time end_hook_t; 
int g_obj_detection_ctr = 0;
long long g_obj_detection_time_acc=0;

void log_data_before_shutting_down(){
    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;

    profiling_data_srv_inst.request.key = "obj_detection_kernel_t";
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


void obj_detect_call_back(const sensor_msgs::ImageConstPtr& msg)
{
    if (result.found){
        result.found = true;
        result.point.x = result.point.x;
        result.point.y = result.point.y;
        return; 
    }

    const double detect_thresh = 0.8;
    cv::Mat depth;
    bounding_box bb = {-3, -3, -3, -3, -3};
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
    
    start_hook_t =  ros::Time::now(); 
    bb = detector.detect_person(cv_ptr->image);
    end_hook_t =  ros::Time::now(); 
    g_obj_detection_ctr++;
    g_obj_detection_time_acc += ((end_hook_t - start_hook_t).toSec()*1e9);

    if(bb.conf >= detect_thresh) {
        ROS_INFO_STREAM("found the object"<< bb.conf);
        /* 
           cv::Mat img_cpy = cv_ptr->image; 
           cv::rectangle(img_cpy, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(0,255,255)); //yellow
           cv::Size size(512, 512);
           cv::imshow(OPENCV_WINDOW, img_cpy);
           cv::waitKey(3);
           */
       
        result.found = true;
        result.point.x = bb.x;
        result.point.y = bb.y;
    }else {
        result.found = false; 
    }
    
    return;
}           

int main(int argc, char** argv){
  ros::init(argc, argv, "object_detection_node");
  ros::NodeHandle nh;
  signal(SIGINT, sigIntHandlerPrivate);

  std::string ns = ros::this_node::getName();
  if (!ros::param::get("/mav_name", mav_name__global)) {
    ROS_FATAL("Could not start object detection node. Parameter missing! Looking for %s",
              (ns + "/mav_name").c_str());
    return -1; 
  }

  if(!ros::param::get("/stats_file_addr",stats_file_addr)){
      ROS_FATAL("Could not start sar. Parameter missing! Looking for %s", 
              (ns + "/stats_file_addr").c_str());
  }

  ros::Publisher obj_det_pub = nh.advertise <mapping_and_sar::OD>("/OD_topic", 4);
  ros::Subscriber raw_image_sub  = nh.subscribe("/Airsim/right/image_raw", 1, obj_detect_call_back);
  
  uint16_t port = 41451;
  std::string ip_addr__global;
  result.found = false;
  
  if (!ros::param::get("/ip_addr", ip_addr__global)) {
    ROS_FATAL_STREAM("Could not start sar. Parameter missing! Looking for /ip_addr");
    return -1;
  }
  
  ros::Rate r(10); 
  while (ros::ok()) {
      obj_det_pub.publish(result);
      ros::spinOnce(); 
      r.sleep();
  }
}


