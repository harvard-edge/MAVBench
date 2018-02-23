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

using namespace std;
bool slam_lost = false;

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

multiDOFpoint current_point(Drone& drone)
{
    multiDOFpoint result;

    geometry_msgs::Pose pose = drone.pose(); // Get drone's current position

    // geometry_msgs::Transform transform;
    // geometry_msgs::Twist velocity;
    // geometry_msgs::Twist acceleration;

    // transform.translation.x = pose.position.x;
    // transform.translation.y = pose.position.y;
    // transform.translation.z = pose.position.z;
    // transform.rotation = pose.orientation;

    // result.transforms.push_back(transform);
    // result.velocities.push_back(velocity);
    // result.accelerations.push_back(acceleration);
    
    result.x = pose.position.x;
    result.y = pose.position.y;
    result.z = pose.position.z;
    result.yaw = YAW_UNCHANGED;
    
    return result;
}

void callback_trajectory(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg, Drone * drone, trajectory_t * normal_traj)
{
    if (msg->points.empty())
        return;

    double dt;
    std::string ns = ros::this_node::getName();
    if (!ros::param::get(ns + "/nbvp/dt", dt)) {
        ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
                (ns + "/nbvp/dt").c_str());
    }

    if (normal_traj->empty()) {
        // Add an initial point at the current location
        multiDOFpoint p = current_point(*drone);
        normal_traj->push_back(p);
    }

    for (const auto& p : msg->points) {
        multiDOFpoint& mdp_prev = normal_traj->back();

        multiDOFpoint mdp_next;
        mdp_next.x = p.transforms[0].translation.x;
        mdp_next.y = p.transforms[0].translation.y;
        mdp_next.z = p.transforms[0].translation.z;
        mdp_next.yaw = YAW_UNCHANGED;

        mdp_prev.vx = (mdp_next.x - mdp_prev.x) / dt;
        mdp_prev.vy = (mdp_next.y - mdp_prev.y) / dt;
        mdp_prev.vz = (mdp_next.z - mdp_prev.z) / dt;
        mdp_prev.yaw = yawFromQuat(p.transforms[0].rotation);
        mdp_prev.duration = dt;

        normal_traj->push_back(mdp_next);
    }
}

bool trajectory_done(const trajectory_t& trajectory) {
    return trajectory.size() == 0;
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

    // Flight queues
    trajectory_t normal_traj, rev_normal_traj;
    trajectory_t slam_loss_traj;

    bool created_slam_loss_traj = false;

    // Connect to drone
    uint16_t port = 41451;
    Drone drone(ip_addr.c_str(), port, localization_method);

    // Subscribe to topics
    std::string topic_name =  mav_name + "/" + mav_msgs::default_topics::COMMAND_TRAJECTORY;
    ros::Subscriber trajectory_follower_sub = n.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(topic_name, 100, boost::bind(callback_trajectory, _1, &drone, &normal_traj));
	ros::Subscriber slam_lost_sub = 
		n.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
    
    // Spin loop
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        trajectory_t * forward_traj = nullptr;
        trajectory_t * rev_traj = nullptr;
        bool check_position = true;
        yaw_strategy_t yaw_strategy = follow_yaw;

        // Handle SLAM loss queue
        if (slam_lost) {
            ROS_WARN("SLAM lost!");
            if (!created_slam_loss_traj)
                slam_loss_traj = create_slam_loss_trajectory(drone, normal_traj, rev_normal_traj);

            created_slam_loss_traj = true;
        } else {
            slam_loss_traj.clear();
            created_slam_loss_traj = false;
        }

        // Choose correct queue to use
        if (slam_lost) {
            forward_traj = &slam_loss_traj;
            rev_traj = &normal_traj;
            check_position = false;
        } else {
            // ROS_INFO("Chose normal path");
            forward_traj = &normal_traj;
            rev_traj = &rev_normal_traj;
        }

        // to keep spinning while hovering to maximize coverage (increasing visibility)
        static bool started_planning = false; 
        if (forward_traj->size() > 0) {
            started_planning = true;
        }       
        if (forward_traj->size() == 0 && started_planning) {//if no trajectory recieved,
                                                            //spin
            int angle = drone.get_yaw()+ 10;
            drone.set_yaw(angle <= 180 ? angle : angle - 360);
        }       

        follow_trajectory(drone, forward_traj, rev_traj, yaw_strategy, check_position);

        // Choose next state (failure, completion, or more flying)
        if (slam_lost && created_slam_loss_traj && trajectory_done(slam_loss_traj))
            reset_slam(drone, "/slam_lost");
        else if (trajectory_done(*forward_traj))
            loop_rate.sleep();
    }

    return 0;
}

