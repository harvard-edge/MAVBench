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
static const std::string OPENCV_WINDOW = "Image window";
static std::string mav_name__global;
YOLODetector detector;
std::string stats_file_addr;
static mapping_and_sar::OD result;

void obj_detect_call_back(const sensor_msgs::ImageConstPtr& msg)
{
    if (result.found){
        result.found = true;
        result.point.x = result.point.x;
        result.point.y = result.point.y;
        return; 
    }

    const double detect_thresh = 0.8;
    //cv::Mat img;
    cv::Mat depth;
    bounding_box bb = {-3, -3, -3, -3, -3};
    //img = drone.read_frame();
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
    
    //ROS_INFO_STREAM("before detection");	
        //ROS_INFO_STREAM("blah blah");
    bb = detector.detect_person(cv_ptr->image);
    //ROS_INFO_STREAM("after detection");	
    //mapping_and_sar::OD result; 
    

    if(bb.conf >= detect_thresh) {
        ROS_INFO_STREAM("found the object"<< bb.conf);
        //update_stats_file(stats_file_addr,"mission_status completed");
        //cv::Mat img_to_show;
        //cv::Mat img_cpy = cv_ptr->image; 
        //cv::rectangle(img_cpy, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(0,255,255)); //yellow
        //cv::Size size(512, 512);
        //resize(img_cpy, img_to_show, size);
        /* 
        for (int i = 0; i < 10000; i++){ 
            //system(("rosnode kill " + mav_name__global + "/sar").c_str());
            //cv::imshow(OPENCV_WINDOW, img_to_show);
            cv::waitKey(3);
        }
        */
        result.found = true;
        result.point.x = bb.x;
        result.point.y = bb.y;
    }else {
        result.found = false; 
    }
    
    //ROS_INFO_STREAM("here is the stuff result.found"<< int(result.found)); 
    return;
}           



int main(int argc, char** argv){
  ros::init(argc, argv, "object_detection_node");
  ros::NodeHandle nh;
  
  ROS_INFO_STREAM("starting object detection"); 
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
  
  //std::string ns = ros::this_node::getName();
  if (!ros::param::get("/ip_addr", ip_addr__global)) {
    ROS_FATAL_STREAM("Could not start sar. Parameter missing! Looking for /ip_addr");
    return -1;
  }
  result.found = false;
  ros::Rate r(10); 
  while (ros::ok()) {
      /* 
      if (result.found) {
          ros::shutdown(); 
      }
      */
      obj_det_pub.publish(result);
      ros::spinOnce(); 
      r.sleep();
  }
}


