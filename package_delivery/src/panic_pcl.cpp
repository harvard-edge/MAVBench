#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "common.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
bool too_close;


void callback(const PointCloud::ConstPtr& msg)//, bool &too_close)
{
    
    float panic_distance;
    ros::param::get("/panic_pcl/panic_distance", panic_distance);
    too_close = false;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
      if (!(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))) {
          if (distance(pt.x, pt.y, pt.z) < panic_distance) {
              too_close = true;
    //          ROS_INFO("\t(%f, %f, %f %d)\n", pt.x, pt.y, pt.z, too_close);
              break;
          }
      }
  }
}


int main(int argc, char** argv)
{
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    std_msgs::Bool panic_msg;
    ros::init(argc, argv, "panic_pcl");
    ros::NodeHandle nS;
    ros::NodeHandle nP;
    //ros::Subscriber sub = nS.subscribe<PointCloud>("points", 1, boost::bind(callback, _1, too_close));
    ros::Subscriber sub = nS.subscribe<PointCloud>("points", 1, callback);
    ros::Publisher panic_publisher = nP.advertise<std_msgs::Bool>("panic_topic", 1000);
    
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce(); 	
        panic_msg.data = too_close;  
        panic_publisher.publish(panic_msg);
        loop_rate.sleep();
  }
}

