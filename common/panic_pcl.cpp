#include "ros/ros.h"

// Standard headers
#include <math.h>
#include <stdio.h>

// PointCloud headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// ROS message headers
#include "std_msgs/Bool.h"

// MAVBench headers
#include "common.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

float panic_distance;

void callback(const PointCloud::ConstPtr& msg, ros::Publisher * panic_pub)
{
    std_msgs::Bool panic_msg;

    for (const pcl::PointXYZ& pt : msg->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
            if (distance(pt.x, pt.y, pt.z) < panic_distance) {
                panic_msg.data = true;
                panic_pub->publish(panic_msg);
                return;
            }
        }
    }

    panic_msg.data = false;
    panic_pub->publish(panic_msg);
}

int main(int argc, char** argv)
{
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    ros::init(argc, argv, "panic_pcl");
    ros::NodeHandle nh;

    ros::Publisher panic_publisher = nh.advertise<std_msgs::Bool>("panic_topic", 1000);
    ros::Subscriber sub = nh.subscribe<PointCloud>("points", 1, boost::bind(callback, _1, &panic_publisher));

    ros::param::get("/panic_pcl/panic_distance", panic_distance);

    ros::spin();
}

