#include "ros/ros.h"
#include <signal.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <stdio.h>
#include "common.h"
#include "Drone.h"
#include <iostream>
#include <fstream>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <mavbench_msgs/multiDOFpoint.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/future_collision.h>
//#include <geometry_msgs/Accel.h>

using namespace std;

// Trajectories
trajectory_t trajectory;
trajectory_t reverse_trajectory;
trajectory_t * traj_to_follow;
trajectory_t * reverse_of_traj_to_follow;

// Messages from other nodes
bool slam_lost = false;
bool col_coming = false;

// Parameters
float g_v_max;
double g_fly_trajectory_time_out;
long long g_planning_time_including_ros_overhead_acc  = 0;
float g_max_yaw_rate;
float g_max_yaw_rate_during_flight;
bool g_trajectory_done = false;
bool g_got_new_trajectory = false;
ros::Time g_recieved_traj_t;

// Profiling
std::string g_supervisor_mailbox; //file to write to when completed
ros::Time col_coming_time_stamp;
float g_localization_status = 1.0;
long long g_rcv_traj_to_follow_traj_acc_t = 0;
bool prev_col_coming = false;
bool CLCT_DATA, DEBUG;
int g_follow_ctr = 0;
long long g_img_to_follow_acc = 0;
ros::Time g_msg_time_stamp;
long long g_pt_cld_to_futurCol_commun_acc = 0;
int g_traj_ctr = 0; 

void col_imminent_callback(const std_msgs::Bool::ConstPtr& msg) {
    //col_imminent = false;
    //ROS_INFO_STREAM("in collision iminent"); 
    col_imminent = msg->data;
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

    profiling_data_srv_inst.request.key = "img_to_follow_traj_commun_t";
    profiling_data_srv_inst.request.value = (((double)g_pt_cld_to_futurCol_commun_acc)/1e9)/g_traj_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "traj_rcv_to_follow";
    profiling_data_srv_inst.request.value = (((double)g_rcv_traj_to_follow_traj_acc_t)/1e9)/g_follow_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "image_to_follow_time";
    profiling_data_srv_inst.request.value = (((double)g_img_to_follow_acc)/1e9)/g_follow_ctr;
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

void callback_trajectory(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg)
{
    if (CLCT_DATA){ 
        g_recieved_traj_t = ros::Time::now();  
        g_msg_time_stamp = msg->header.stamp;
        if (g_msg_time_stamp.sec != 0) {  
            g_pt_cld_to_futurCol_commun_acc += (ros::Time::now() - msg->header.stamp).toSec()*1e9;
            g_traj_ctr++; 
        } 
    }

    normal_traj.clear(); 
    for (auto point : msg->points){
        multiDOFpoint traj_point;
        traj_point.x = point.x;
        traj_point.y = point.y;
        traj_point.z = point.z;
        traj_point.vx = point.vx;
        traj_point.vy = point.vy;
        traj_point.vz = point.vz;
        traj_point.yaw = point.yaw;
        traj_point.duration = point.duration;
        normal_traj.push_back(traj_point);
    }

    g_got_new_trajectory = true;
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

bool follow_trajectory_status_cb(package_delivery::follow_trajectory_status_srv::Request &req, 
    package_delivery::follow_trajectory_status_srv::Response &res)
{
    res.success.data = g_trajectory_done;// || col_coming;

    const multiDOFpoint& current_point =
        normal_traj.empty() ? rev_normal_traj.front() : normal_traj.front();
    
    geometry_msgs::Twist last_velocity;
    last_velocity.linear.x = current_point.vx;
    last_velocity.linear.y = current_point.vy;
    last_velocity.linear.z = current_point.vz;
    res.twist = last_velocity;
  
    geometry_msgs::Twist last_acceleration;
    last_acceleration.linear.x = current_point.ax;
    last_acceleration.linear.y = current_point.ay;
    last_acceleration.linear.z = current_point.az;
    res.acceleration = last_acceleration;
    return true;
}

package_delivery::multiDOF_array next_steps_msg(const trajectory_t& traj) {
    package_delivery::multiDOF_array array_of_point_msg;

    for (const auto& point : traj){
        package_delivery::multiDOF point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.z = point.z;
        point_msg.vx = point.vx;
        point_msg.vy = point.vy;
        point_msg.vz = point.vz;
        point_msg.ax = point.ax;
        point_msg.ay = point.ay;
        point_msg.az = point.az;
        point_msg.yaw = point.yaw;
        point_msg.duration = point.duration;
        array_of_point_msg.points.push_back(point_msg); 
    }

    return array_of_point_msg;
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
    uint16_t port = 41451;
    ros::Rate loop_rate(50);
    
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

    if(!ros::param::get("CLCT_DATA",CLCT_DATA))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory CLCT_DATA not provided");
        return -1;
    }
    if(!ros::param::get("DEBUG",DEBUG))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory DEBUG not provided");
        return -1;
    }
    if(!ros::param::get("/fly_trajectory_time_out", g_fly_trajectory_time_out)){
        ROS_FATAL("Could not start follow_thrajectory. Parameter missing! fly_trajectory_time_out is not provided"); 
        return -1; 
    }
    
    ros::Publisher next_steps_pub = n.advertise<package_delivery::multiDOF_array>("/next_steps", 1);

    Drone drone(ip_addr.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight);

	ros::Subscriber slam_lost_sub = 
		n.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
    ros::Subscriber trajectory_follower_sub = 
        n.subscribe<package_delivery::multiDOF_array>("normal_traj", 
                1, boost::bind(callback_trajectory, _1, &drone));

    bool app_started = false;  //decides when the first planning has occured
                               //this allows us to activate all the
                               //functionaliy in follow_trajecotry accordingly
    while (ros::ok()) {
        ros::spinOnce();

        trajectory_t * forward_traj = nullptr;
        trajectory_t * rev_traj = nullptr;
        bool check_position = true;
        yaw_strategy_t yaw_strategy = follow_yaw;

        if (normal_traj.size() > 0) {
            app_started = true;
        }
       
        if(app_started){
            // Profiling 
            if (CLCT_DATA) { 
                if (g_got_new_trajectory) {
                    g_rcv_traj_to_follow_traj_acc_t +=  
                        (ros::Time::now() - g_recieved_traj_t).toSec()*1e9;
                    if (g_msg_time_stamp.sec != 0) {  
                        g_img_to_follow_acc += (ros::Time::now() - g_msg_time_stamp).toSec()*1e9;
                        g_follow_ctr++; 
                    }
                    if (DEBUG) {
                        //ROS_INFO_STREAM("follow_traj_cb to  func" << g_rcv_traj_to_follow_traj_t);
                        //ROS_INFO_STREAM("whatevs------- " << g_img_to_follow_acc);
                    }
                } 
            }
            
            // Back up if no trajectory was found
            if (!forward_traj->empty())
                follow_trajectory(drone, forward_traj, rev_traj, yaw_strategy, 
                    check_position, g_v_max, g_fly_trajectory_time_out);
            else {
                //ROS_ERROR("New SLAMING BREAKS YO");
                follow_trajectory(drone, &rev_normal_traj, nullptr, face_backward, true, 1, g_fly_trajectory_time_out);
            }

            next_steps_pub.publish(next_steps_msg(*forward_traj));
        }
        if (slam_lost && trajectory_done(slam_loss_traj)){
            ROS_INFO_STREAM("slam loss");
            log_data_before_shutting_down(); 
            g_localization_status = 0; 
            signal_supervisor(g_supervisor_mailbox, "kill"); 
            ros::shutdown();
        }else if (trajectory_done(*forward_traj)){
            loop_rate.sleep();
        }
        g_got_new_trajectory = false;
    }
    return 0;
}

