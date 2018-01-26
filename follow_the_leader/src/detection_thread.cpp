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

void sample_images_cb(const sensor_msgs::ImageConstPtr& msg) {
      cv_img = *cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      img_id = cv_img.header.seq;

}

bool detection_cb(follow_the_leader::cmd_srv::Request &req, 
    follow_the_leader::cmd_srv::Response &res)
{
 
    
    //cv::Mat img;
    cv::Mat depth;
    bounding_box bb = {-3, -3, -3, -3, -3};
    //img = drone.read_frame();
    
    cv::Mat img_cpy = cv_img.image; 
    
    //--- inflating the img 
    //cv::Size size(1024, 576);
    
    //cv::Size size(256, 256);
    //cv::Size size(512, 288);
    //cv::Size size(1024, 576);
    //cv::Mat img_inflated; //required since detection has a lower limit on the size 
                          // of the object
    //resize(img_cpy, img_inflated, size);

    steady_clock::time_point detec_t_s; //total function time s
    steady_clock::time_point detec_t_e; //total function time s
    
    detec_t_s= steady_clock::now();
    bb = detector.detect_person(img_cpy);
    detec_t_e = steady_clock::now();
    auto det__t = duration_cast<milliseconds>(detec_t_e- detec_t_s).count();
    file_to_output<<"detection time:"<<det__t<<endl;
     /*
    bb.x = bb.x*(512.0/1024.0);
    bb.y = bb.y*(288.0/1024.0);
    bb.w = bb.w*(512.0/1024.0);
    bb.h = bb.h*(288.0/1024.0);
    */
    /*
    bb.x = bb.x*(1024.0/1024.0);
    bb.y = bb.y*(576.0/1024.0);
    bb.w = bb.w*(1024.0/1024.0);
    bb.h = bb.h*(576.0/1024.0);
*/
    //showing the result 
    cv::Mat img_cpy_2 = img_cpy; 
    //cv::Mat img_cpy_2 = img_inflated; 
    cv::rectangle(img_cpy_2, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,255,0)); //yellow
    //cv::imshow(OPENCV_WINDOW, img_inflated);
    

    if(bb.conf >= detect_thresh) {
        //ROS_INFO_STREAM("found the object"<< bb.conf);
       cv::imshow(OPENCV_WINDOW, img_cpy_2);
       //cv::waitKey(40);

   
       /* 
        cv::Mat img_to_show;
        cv::Mat img_cpy = cv_img.image; 
        cv::rectangle(img_cpy, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,255,0)); //yellow
        
        //cv::imshow(OPENCV_WINDOW, cv_img.image);
        //cv::waitKey(50);
*/
        res.status= "obj_detected";
        res.bb.x = bb.x;
        res.bb.y = bb.y;
        res.bb.w = bb.w;
        res.bb.h = bb.h;
        res.bb.conf = bb.conf;
        res.img_id = img_id;
       
        //ROS_INFO_STREAM("bb in detector"<<res.bb.x<<" " <<res.bb.y<< " " << res.bb.w << " " <<res.bb.h);
        
        //cv::destroyAllWindows();
    }else {
        res.status = "resume_detection"; 
        //ROS_INFO_STREAM("object couldn't not be found"<< bb.conf);
    }
    return true;
}           

int main(int argc, char** argv)
{
    
    file_to_output.open("/home/nvidia/catkin_ws/src/mav-bench/follow_the_leader/src/detection_output.txt");
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

    if (!ros::param::get("/detect_thresh", detect_thresh)) {
        ROS_FATAL_STREAM("Could not start detection. Parameter missing! Looking for /detect_thresh");
        return -1;
    }
    /* 
    if(!ros::param::get("/stats_file_addr",stats_file_addr)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                "/stats_file_addr");
     return -1; 
    }
*/
    ros::Subscriber raw_image_sub  = nh.subscribe("/Airsim/right/image_raw", 1, sample_images_cb);
    ros::ServiceServer detect_server = 
        nh.advertiseService("detect", detection_cb);

    //ros::ServiceClient img_id_client = 
     //   nh.serviceClient<follow_the_leader::img_id>("img_id");



    //int detection_loop_rate = 10;
    //ros::Rate loop_rate(detection_loop_rate);

    //update_stats_file(stats_file_addr,"inside before rosspin");
    while(ros::ok) {
        ros::spinOnce();
    }
}
