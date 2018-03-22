#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <signal.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "common.h"
#include <mav_msgs/default_topics.h>
#include "Drone.h"
#include <iostream>
#include <fstream>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <std_srvs/SetBool.h>
#include <package_delivery/multiDOF.h>
#include <package_delivery/multiDOF_array.h>
#include <package_delivery/follow_trajectory_status_srv.h>
#include <package_delivery/BoolPlusHeader.h>
//#include <geometry_msgs/Accel.h>
using namespace std;
bool slam_lost = false;
bool col_imminent = false;
//bool col_coming = false;


trajectory_t normal_traj;
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
bool g_got_new_trajectory = false;
ros::Time g_recieved_traj_t;

// Profiling
ros::Time col_coming_time_stamp;
ros::Duration g_rcv_traj_to_follow_traj_t;
bool prev_col_coming = false;
bool CLCT_DATA, DEBUG;


void panic_callback(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
}

void panic_velocity_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    panic_velocity = *msg;
}

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
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

void callback_trajectory(const package_delivery::multiDOF_array::ConstPtr& msg, Drone * drone)//, trajectory_t * normal_traj)
{
    g_recieved_traj_t = ros::Time::now();  
    //g_recieved_traj_t = ms->header.stamp;
    normal_traj.clear(); 
    if(DEBUG) { 
        ROS_INFO_STREAM("pkg_del to call_back traj"<< ros::Time::now() - msg->header.stamp);
    }
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
    //ROS_INFO_STREAM("finished trajectory, size is"<<normal_traj.size());  
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
    geometry_msgs::Twist last_velocity;
    last_velocity.linear.x = normal_traj.front().vx;
    last_velocity.linear.y = normal_traj.front().vy;
    last_velocity.linear.z = normal_traj.front().vz;
    res.twist = last_velocity;
  
    geometry_msgs::Twist last_acceleration;
    last_acceleration.linear.x = normal_traj.front().ax;
    last_acceleration.linear.y = normal_traj.front().ay;
    last_acceleration.linear.z = normal_traj.front().az;
    res.acceleration = last_acceleration;
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
    trajectory_t rev_normal_traj, panic_traj;
    trajectory_t slam_loss_traj;
    bool created_slam_loss_traj = false;
    bool created_future_col_traj = false;
    trajectory_t future_col_traj;
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


/*
    if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory supervisor_mailbox not provided");
        return -1;
    }
*/
    if(!ros::param::get("/fly_trajectory_time_out", g_fly_trajectory_time_out)){
        ROS_FATAL("Could not start follow_thrajectory. Parameter missing! fly_trajectory_time_out is not provided"); 
        return -1; 
    }
    
    ros::ServiceServer trajectory_done_service = n.advertiseService("follow_trajectory_status", follow_trajectory_status_cb);
    ros::Subscriber panic_sub =  n.subscribe<std_msgs::Bool>("panic_topic", 1, panic_callback);
    ros::Subscriber panic_velocity_sub = 
        n.subscribe<geometry_msgs::Vector3>("panic_velocity", 1, panic_velocity_callback);
    

    
    Drone drone(ip_addr.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight);
	ros::Subscriber slam_lost_sub = 
		n.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
    ros::Subscriber trajectory_follower_sub = 
        n.subscribe<package_delivery::multiDOF_array>("normal_traj", 
                1, boost::bind(callback_trajectory, _1, &drone));//, &normal_traj));

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
            // ROS_ERROR("Panicking!");
            panic_traj = create_panic_trajectory(drone, panic_velocity);
            normal_traj.clear(); // Replan a path once we're done
        } else {
            panic_traj.clear();
        }

        if (slam_lost) {
            ROS_WARN("SLAM lost!");
            if (!created_slam_loss_traj)
                slam_loss_traj = create_slam_loss_trajectory(drone, normal_traj, rev_normal_traj);

            created_slam_loss_traj = true;
        } else {
            slam_loss_traj.clear();
            created_slam_loss_traj = false;
        }

        // Handle future_collision queue
         /*
        if (col_coming) {
            ROS_WARN("Future collision appeared on trajectory!");
        
            if (!created_future_col_traj)
                future_col_traj = create_future_col_trajectory(normal_traj, 0.5);

            created_future_col_traj = true;

            ROS_WARN_STREAM("Future col length " << future_col_traj.size());

            normal_traj.clear(); // Replan the normal path once we're done
        } else {
            future_col_traj.clear();
            created_future_col_traj = false;
        }
        */
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

        
        if (normal_traj.size() > 0) {
            app_started = true;
        }       
       
        if(app_started){
            // Profiling 
             
            if (CLCT_DATA) { 
                if (g_got_new_trajectory) {
                    g_rcv_traj_to_follow_traj_t =  
                        ros::Time::now() - g_recieved_traj_t;
                    if (DEBUG) {
                        ROS_INFO_STREAM("follow_traj_cb to  func" << g_rcv_traj_to_follow_traj_t);
                    }
                } 
            }
            
            follow_trajectory(drone, forward_traj, rev_traj, yaw_strategy, 
                    check_position, g_v_max, g_fly_trajectory_time_out);
        }
        if (slam_lost && created_slam_loss_traj && trajectory_done(slam_loss_traj)){
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

