#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "common.h"
#include "Drone.h"
#include <std_msgs/Bool.h>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/future_collision.h>

using namespace std;

// Trajectories
trajectory_t trajectory;
trajectory_t reverse_trajectory;
bool fly_backward = false;

// Messages from other nodes
bool slam_lost = false;
// ros::Time future_collision_time{0};
int future_collision_seq = 0;
int trajectory_seq = 0;

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

void future_collision_callback(const mavbench_msgs::future_collision::ConstPtr& msg) {
    if (msg->header.seq > future_collision_seq)
        future_collision_seq = msg->header.seq;
    // future_collision_time = ros::Time::now() + ros::Duration(time_to_collision);
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

void callback_trajectory(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg)
{
    if (msg->header.seq < trajectory_seq)
        return;
    else
        trajectory_seq = msg->header.seq;

    // Check for trajectories that are not updated to the latest collision
    // detection, or that arrive out of order
    if (msg->future_collision_seq < future_collision_seq)
        return;

    if (CLCT_DATA){
        g_recieved_traj_t = ros::Time::now();
        g_msg_time_stamp = msg->header.stamp;
        if (g_msg_time_stamp.sec != 0) {
            g_pt_cld_to_futurCol_commun_acc += (ros::Time::now() - msg->header.stamp).toSec()*1e9;
            g_traj_ctr++;
        } 
    }

    if (msg->reverse) {
        fly_backward = true;
    } else {
        trajectory_t new_trajectory = create_trajectory_from_msg(*msg);

        if (msg->append)
            trajectory = append_trajectory(trajectory, new_trajectory);
        else
            trajectory = new_trajectory;

        fly_backward = false;
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


void initialize_global_params() {
    if(!ros::param::get("v_max",g_v_max))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory vmax not provided");
        exit(-1);
    }

    if(!ros::param::get("max_yaw_rate",g_max_yaw_rate))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate not provided");
        exit(-1);
    }

    if(!ros::param::get("max_yaw_rate_during_flight",g_max_yaw_rate_during_flight))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate_during_flight not provided");
        exit(-1);
    }

    if(!ros::param::get("CLCT_DATA",CLCT_DATA))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory CLCT_DATA not provided");
        exit(-1);
    }
    if(!ros::param::get("DEBUG",DEBUG))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory DEBUG not provided");
        exit(-1);
    }
    if(!ros::param::get("/fly_trajectory_time_out", g_fly_trajectory_time_out)){
        ROS_FATAL("Could not start follow_thrajectory. Parameter missing! fly_trajectory_time_out is not provided");
        exit(-1);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_trajectory", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandlerPrivate);

    ros::Rate loop_rate(50);

    // Initialize the drone
    std::string localization_method;
    std::string mav_name;
    std::string ip_addr;
    const uint16_t port = 41451;

    ros::param::get("/follow_trajectory/ip_addr", ip_addr);
    ros::param::get("/follow_trajectory/mav_name", mav_name);
    ros::param::get("/follow_trajectory/localization_method", localization_method);
    Drone drone(ip_addr.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight);

    // Initialize publishers and subscribers
    ros::Publisher next_steps_pub = n.advertise<mavbench_msgs::multiDOFtrajectory>("/next_steps", 1);

    n.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
    n.subscribe<mavbench_msgs::future_collision>("/col_coming", 1, future_collision_callback);
    n.subscribe<mavbench_msgs::multiDOFtrajectory>("normal_traj", 1, callback_trajectory);

    // Begin execution loop
    bool app_started = false;  //decides when the first planning has occured
                               //this allows us to activate all the
                               //functionaliy in follow_trajectory accordingly
    while (ros::ok()) {
        ros::spinOnce();

        if (trajectory.size() > 0) {
            app_started = true;
        }

        if (app_started) {
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
        }

        // Figure out which direction we will fly in
        trajectory_t * forward_traj;
        trajectory_t * rev_traj;

        yaw_strategy_t yaw_strategy = follow_yaw;
        float max_velocity = g_v_max;

        if (!fly_backward) {
            forward_traj = &trajectory;
            rev_traj = &reverse_trajectory;
        } else {
            forward_traj = &reverse_trajectory;
            rev_traj = &trajectory;

            yaw_strategy = face_backward;
            max_velocity = 1;
        }

        follow_trajectory(drone, forward_traj, rev_traj, yaw_strategy,
            true, max_velocity, g_fly_trajectory_time_out);

        // Publish the remainder of the trajectory
        mavbench_msgs::multiDOFtrajectory trajectory_msg = create_trajectory_msg(*forward_traj);
        trajectory_msg.header.seq = trajectory_seq;

        next_steps_pub.publish(trajectory_msg);

        if (slam_lost){
            ROS_INFO_STREAM("slam loss");
            log_data_before_shutting_down();
            g_localization_status = 0;
            signal_supervisor(g_supervisor_mailbox, "kill");
            ros::shutdown();
        } else if (trajectory_done(*forward_traj)){
            loop_rate.sleep();
        }

        g_got_new_trajectory = false;
    }
    return 0;
}

