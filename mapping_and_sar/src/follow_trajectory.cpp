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
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <std_srvs/SetBool.h>

using namespace std;
bool slam_lost = false;
float g_localization_status = 1.0;
std::string g_supervisor_mailbox; //file to write to when completed
float g_v_max;
double g_fly_trajectory_time_out;
long long g_planning_time_including_ros_overhead_acc  = 0;

float g_max_yaw_rate;
float g_max_yaw_rate_during_flight;
bool g_trajectory_done = false;
bool should_panic = false;
geometry_msgs::Vector3 panic_velocity;

//bool g_dummy = false;

void panic_callback(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
}

void panic_velocity_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    panic_velocity = *msg;
}

void log_data_before_shutting_down(){
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profiling_data_srv_inst.request.key = "localization_status";
    profiling_data_srv_inst.request.value = g_localization_status;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

/*
void dummy_cb(const std_msgs::Bool::ConstPtr& msg) {
    g_dummy = msg->data; 
    //slam_lost = msg->data;
}
*/

multiDOFpoint current_point(Drone& drone){
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
    result.vx = 0;
    result.vy = 0;
    result.vz = 0;
    
    return result;
}


void callback_trajectory(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg, Drone * drone, trajectory_t * normal_traj)
{
    if (msg->points.empty())
        return;

    g_trajectory_done = false;
    
    double dt;
    std::string ns = ros::this_node::getName();
    if (!ros::param::get(ns + "/nbvp/dt", dt)) {
        ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
                (ns + "/nbvp/dt").c_str());
    }

    //--first correct  
    bool correct = false; //only used for the first point
    multiDOFpoint mdp_prev; 
    multiDOFpoint mdp_next;
    mdp_prev = current_point(*drone);
    
    float correction_in_x = msg->points[0].transforms[0].translation.x - mdp_prev.x;
    float correction_in_y = msg->points[0].transforms[0].translation.y - mdp_prev.y;
    float correction_in_z =  msg->points[0].transforms[0].translation.z - mdp_prev.z;
    float first_yaw = yawFromQuat(msg->points[0].transforms[0].rotation);
    ROS_INFO_STREAM("correction values"<<correction_in_x << " " <<correction_in_y << " "<<correction_in_z);
    double disc = min((dt * g_v_max) / 
        distance(mdp_prev.x - msg->points[0].transforms[0].translation.x,
                mdp_prev.y - msg->points[0].transforms[0].translation.y,
                mdp_prev.z - msg->points[0].transforms[0].translation.z), 1.0);
    
    for (double it = disc; it <= 1.0; it += disc) {
        mdp_next.x = mdp_prev.x + disc*(correction_in_x);
        mdp_next.y = mdp_prev.y + disc*correction_in_y;
        mdp_next.z = mdp_prev.z + disc*correction_in_z;
        mdp_next.yaw = YAW_UNCHANGED;

        mdp_next.vx = (mdp_next.x - mdp_prev.x) / dt;
        mdp_next.vy = (mdp_next.y - mdp_prev.y) / dt;
        mdp_next.vz = (mdp_next.z - mdp_prev.z) / dt;
        mdp_next.yaw = first_yaw;
        mdp_next.duration = dt;
        normal_traj->push_back(mdp_next);
        mdp_prev = mdp_next;
    }
        
    //after correction, now push the new points 
    for (const auto& p : msg->points) {
        if (normal_traj->size() == 0) {
            
            mdp_prev = current_point(*drone);
        } else{
            mdp_prev = normal_traj->back();
        }

        mdp_next.x = p.transforms[0].translation.x;
        mdp_next.y = p.transforms[0].translation.y;
        mdp_next.z = p.transforms[0].translation.z;
        //mdp_next.yaw = YAW_UNCHANGED;


        mdp_next.vx = (mdp_next.x - mdp_prev.x) / dt;
        mdp_next.vy = (mdp_next.y - mdp_prev.y) / dt;
        mdp_next.vz = (mdp_next.z - mdp_prev.z) / dt;
        mdp_next.yaw = yawFromQuat(p.transforms[0].rotation);
        mdp_next.duration = dt;
        normal_traj->push_back(mdp_next);
    
        correct = false;
    }
}

bool trajectory_done(const trajectory_t& trajectory) {
    trajectory.size() == 0;
    g_trajectory_done = (trajectory.size() == 0);
    return g_trajectory_done;
}

void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        signal_supervisor(g_supervisor_mailbox, "kill"); 
        ros::shutdown();
    }
    exit(0);
}


bool trajectory_done_srv_cb(std_srvs::SetBool::Request &req, 
    std_srvs::SetBool::Response &res)
{
    res.success = g_trajectory_done;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "follow_trajectory", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandlerPrivate);
    std::string localization_method; 
    std::string mav_name;
    std::string ip_addr;
    ros::Time cur_t, last_t;
    float cur_z, last_z = -9999;
    trajectory_t normal_traj, rev_normal_traj, panic_traj;
    trajectory_t slam_loss_traj;
    bool created_slam_loss_traj = false;
    uint16_t port = 41451;
    ros::Rate loop_rate(20);
    
    ros::param::get("/follow_trajectory/ip_addr",ip_addr);
    ros::param::get("/follow_trajectory/mav_name",mav_name);
    if(!ros::param::get("/follow_trajectory/localization_method",localization_method))  {
        ROS_FATAL_STREAM("Could not start follow trajectory cause localization_method not provided");
        return -1; 
    }

    if(!ros::param::get("v_max",g_v_max))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory vmax not provided");
        return -1;
    }

    if(!ros::param::get("max_yaw_rate",g_max_yaw_rate))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate not provided");
        return -1;
    }

    if(!ros::param::get("max_yaw_rate_during_flight",g_max_yaw_rate_during_flight))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate_during_flight not provided");
        return -1;
    }

    if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory supervisor_mailbox not provided");
        return -1;
    }

    if(!ros::param::get("/fly_trajectory_time_out", g_fly_trajectory_time_out)){
        ROS_FATAL("Could not start follow_thrajectory. Parameter missing! fly_trajectory_time_out is not provided"); 
     return -1; 
    }

    
    ros::ServiceServer trajectory_done_service = n.advertiseService("trajectory_done_srv", trajectory_done_srv_cb);
    ros::Subscriber panic_sub =  n.subscribe<std_msgs::Bool>("panic_topic", 1, panic_callback);
    ros::Subscriber panic_velocity_sub = 
        n.subscribe<geometry_msgs::Vector3>("panic_velocity", 1, panic_velocity_callback);
    std::string topic_name =  mav_name + "/" + mav_msgs::default_topics::COMMAND_TRAJECTORY;
    
    Drone drone(ip_addr.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight);
	ros::Subscriber slam_lost_sub = 
		n.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
    ros::Subscriber trajectory_follower_sub = n.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(topic_name, 100, boost::bind(callback_trajectory, _1, &drone, &normal_traj));

    int v_x, v_y, v_z;
    v_z = .6; 
    ros::Time last_time = ros::Time::now();
    bool app_started = false;  //decides when the first planning has occured
                               //this allows us to activate all the
                               //functionaliy in follow_trajecotry accordingly
    while (ros::ok()) {
        ros::spinOnce();

        trajectory_t * forward_traj = nullptr;
        trajectory_t * rev_traj = nullptr;
        bool check_position = true;
        yaw_strategy_t yaw_strategy = follow_yaw;

        if (should_panic) {
            ROS_DEBUG("Panicking!");
            panic_traj = create_panic_trajectory(drone, panic_velocity);
            normal_traj.clear(); // Replan a path once we're done
        } else {
            panic_traj.clear();
        }

        if (slam_lost) {
            ROS_WARN("SLAM lost!");
            if (!created_slam_loss_traj) {
                // slam_loss_traj = create_slam_loss_trajectory(drone, normal_traj, rev_normal_traj);
                slam_loss_traj = trajectory_t();
            }

            created_slam_loss_traj = true;
        } else {
            slam_loss_traj.clear();
            created_slam_loss_traj = false;
        }

        if (!panic_traj.empty()) {
            forward_traj = &panic_traj;
            rev_traj = nullptr;
            check_position = false;
            yaw_strategy = ignore_yaw;
        } else if (!slam_loss_traj.empty()) {
            forward_traj = &slam_loss_traj;
            rev_traj = &normal_traj;
            check_position = false;
        } else {
            forward_traj = &normal_traj;
            rev_traj = &rev_normal_traj;
        }

        
        // to keep spinning while hovering to maximize coverage (increasing visibility)
        if (normal_traj.size() > 0) {
            app_started = true;
        }       
        cur_z = drone.pose().position.z; // Get drone's current position
        // if (forward_traj->size() == 0 && app_started) {//if no trajectory recieved,
        //     double dt = (ros ::Time::now() - last_time).toSec(); 
        //     if (dt > .6) {
        //         if (last_z != -9999){
        //             double vz = (last_z-cur_z)/dt;

        //             drone.fly_velocity(0, 0, vz,
        //                     drone.get_yaw()+30, dt); 
        //         }
        //         //drone.fly_velocity(0*v_x, 0*v_y, v_z, drone.get_yaw()+30, 0.3); 
        //         last_z = drone.pose().position.z; // Get drone's current position
        //         last_time = ros::Time::now();
        //     }
        // }
        
        if(app_started){
            follow_trajectory(drone, forward_traj, rev_traj, yaw_strategy,
                    check_position, g_v_max, g_fly_trajectory_time_out);
        }

        if(forward_traj->size() != 0) {
            last_time = ros::Time::now();
        }
        if (slam_lost && created_slam_loss_traj && trajectory_done(slam_loss_traj)){
            ROS_ERROR("Slam lost without recovery");
            log_data_before_shutting_down();
            g_localization_status = 0;
            signal_supervisor(g_supervisor_mailbox, "kill");
            ros::shutdown();
        }else if (trajectory_done(*forward_traj)){
            loop_rate.sleep();
        }
    }
    return 0;
}

