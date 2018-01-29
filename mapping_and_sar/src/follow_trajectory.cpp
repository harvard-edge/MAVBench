#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <signal.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "common.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/default_topics.h>
#include "Drone.h"
#include <iostream>
#include <fstream>

trajectory_t trajectory, reverse_trajectory;

/*
void print_trajectory(trajectory_t trajectory) {
    // static std::ofstream ofs("/home/ubuntu/log.txt", std::ofstream::app);

    std::cout << "================" << std::endl;
    int i = 0;
    for (auto p : trajectory) {
        std::cout << "---------" << std::endl;
        std::cout << "Entry: " << i++ << std::endl;

        std::cout << "Velocities: {" << std::endl;
        std::cout << "x: " << p.velocities[0].linear.x << std::endl;
        std::cout << "y: " << p.velocities[0].linear.y << std::endl;
        std::cout << "z: " << p.velocities[0].linear.z << std::endl;
        std::cout << "}" << std::endl;

        std::cout << "Positions: {" << std::endl;
        std::cout << "x: " << p.transforms[0].translation.x << std::endl;
        std::cout << "y: " << p.transforms[0].translation.y << std::endl;
        std::cout << "z: " << p.transforms[0].translation.z << std::endl;
        std::cout << "}" << std::endl;

        std::cout << "Rotations: {" << std::endl;
        std::cout << "x: " << p.transforms[0].rotation.x << std::endl;
        std::cout << "y: " << p.transforms[0].rotation.y << std::endl;
        std::cout << "z: " << p.transforms[0].rotation.z << std::endl;
        std::cout << "z: " << p.transforms[0].rotation.w << std::endl;
        std::cout << "}" << std::endl;

        std::cout << "Time: " << p.time_from_start.toSec() << std::endl << std::endl;
    }
    std::cout << "================" << std::endl << std::endl;
}
*/

multiDOFpoint current_point(Drone& drone)
{
    multiDOFpoint result;

    geometry_msgs::Pose pose = drone.pose(); // Get drone's current position

    geometry_msgs::Transform transform;
    geometry_msgs::Twist velocity;
    geometry_msgs::Twist acceleration;

    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation = pose.orientation;

    result.transforms.push_back(transform);
    result.velocities.push_back(velocity);
    result.accelerations.push_back(acceleration);

    return result;
}

void callback_trajectory(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg, Drone *drone)
{
    if (msg->points.empty())
        return;

    if (trajectory.empty()) {
        // Add an initial trajectory point at the current location
        multiDOFpoint p = current_point(*drone);
        p.time_from_start = ros::Duration(0);
        trajectory.push_back(p);
    }

    double dt; 
    std::string ns = ros::this_node::getName();
    if (!ros::param::get(ns + "/nbvp/dt", dt)) {
        ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
                (ns + "/nbvp/dt").c_str());
    }

    for (multiDOFpoint p : msg->points) {
        multiDOFpoint& p_prev = trajectory.back();

        double x = p.transforms[0].translation.x;
        double y = p.transforms[0].translation.y;
        double z = p.transforms[0].translation.z;

        double x_prev = p_prev.transforms[0].translation.x;
        double y_prev = p_prev.transforms[0].translation.y;
        double z_prev = p_prev.transforms[0].translation.z;

        p.time_from_start = p_prev.time_from_start + ros::Duration(dt);

        p_prev.velocities[0].linear.x = (x - x_prev) / dt;
        p_prev.velocities[0].linear.y = (y - y_prev) / dt;
        p_prev.velocities[0].linear.z = (z - z_prev) / dt;

        trajectory.push_back(p);
    }

    // print_trajectory(trajectory);
}

bool trajectory_done(const trajectory_t& trajectory) {
    return trajectory.size() <= 1;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "follow_trajectory", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandler);

    // Read parameters
    std::string localization_method; 
    std::string mav_name;
    std::string ip_addr;

    ros::param::get("/follow_trajectory/ip_addr",ip_addr);
    ros::param::get("/follow_trajectory/mav_name",mav_name);
    if(!ros::param::get("/follow_trajectory/localization_method",localization_method))  {
        ROS_FATAL_STREAM("Could not start follow trajectory cause localization_method not provided");
        return -1; 
    }

    // Connect to drone
    uint16_t port = 41451;
    Drone drone(ip_addr.c_str(), port, localization_method);

    // Subscribe to topics
    std::string topic_name =  mav_name + "/" + mav_msgs::default_topics::COMMAND_TRAJECTORY;
    ros::Subscriber trajectory_follower_sub = n.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(topic_name, 100, boost::bind(callback_trajectory, _1, &drone));
    
    // Spin loop
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (!trajectory_done(trajectory)) {
            follow_trajectory(drone, trajectory, reverse_trajectory, follow_yaw);
        } else
            loop_rate.sleep();
    }

    return 0;
}

