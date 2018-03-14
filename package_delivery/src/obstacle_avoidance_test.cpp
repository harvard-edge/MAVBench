#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <iostream>
#include <chrono>
#include <cmath>
#include <chrono>
#include <thread>
#include <deque>
#include <limits>
#include <signal.h>

#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include "control_drone.h"
#include "common/Common.hpp"
#include "Drone.h"
#include "package_delivery/get_trajectory.h"
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include "common.h"
#include "timer.h"

using namespace std;

//global variable to log in stats manager
std::string g_mission_status = "failed";

bool should_panic = false;
bool col_imminent = false;
bool col_coming = false;
bool slam_lost = false;

long long g_accumulate_loop_time = 0; //it is in ms
long long g_panic_rlzd_t_accumulate = 0;
int g_main_loop_ctr = 0;
int g_panic_ctr = 0;
bool g_start_profiling = false; 

double v_max__global, a_max__global, g_fly_trajectory_time_out;
long long g_planning_time_including_ros_overhead_acc  = 0;
int  g_planning_ctr = 0; 
bool clct_data = true;

geometry_msgs::Vector3 panic_velocity;
string ip_addr__global;
string localization_method;
string stats_file_addr;
string ns;
std::string g_supervisor_mailbox; //file to write to when completed
int g_hand_off_control_time; //file to write to when completed

enum State { setup, waiting, flying, completed, failed, invalid };

void log_data_before_shutting_down(){

    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    profiling_data_srv_inst.request.key = "mission_status";
    profiling_data_srv_inst.request.value = (g_mission_status == "completed" ? 1.0: 0.0);
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "obstacle_avoidance_test_main_loop";
    profiling_data_srv_inst.request.value = (((double)g_accumulate_loop_time)/1e9)/g_main_loop_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
        }
    }

    profiling_data_srv_inst.request.key = "panic_ctr";
    profiling_data_srv_inst.request.value = g_panic_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "panic_time_in_obstacle_avoidance_test_node";
    profiling_data_srv_inst.request.value = (g_panic_rlzd_t_accumulate/ (double)g_panic_ctr)*1e-9;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "motion_planning_plus_srv_call";
    profiling_data_srv_inst.request.value = ((double)g_planning_time_including_ros_overhead_acc/g_planning_ctr)/1e9;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
}

void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        ros::shutdown();
    }
    exit(0);
}

double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.x)*(t.x-m.x) + (t.y-m.y)*(t.y-m.y) + (t.z-m.z)*(t.z-m.z));
}

// *** F:DN call back function for the panic_topic subscriber
void panic_callback(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
}

void panic_velocity_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    panic_velocity = *msg;
}

void col_imminent_callback(const std_msgs::Bool::ConstPtr& msg) {
    col_imminent = msg->data;
}

void col_coming_callback(const std_msgs::Bool::ConstPtr& msg) {
    col_coming = msg->data;
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

void obstacle_avoidance_test_initialize_params() {
    if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
      ROS_FATAL_STREAM("Could not start mapping supervisor_mailbox not provided");
      return ;
    }
    
    if(!ros::param::get("/hand_off_control_time",g_hand_off_control_time))  {
      ROS_FATAL_STREAM("Could not start obstacle_avoidance_test hand_off_control_time not provided");
      return ;
    }

    if(!ros::param::get("/obstacle_avoidance_test/ip_addr",ip_addr__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/ip_addr").c_str());
      return; 
    }
    if(!ros::param::get("/obstacle_avoidance_test/localization_method",localization_method)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/localization_method").c_str());
       return; 
    }
    if(!ros::param::get("/stats_file_addr",stats_file_addr)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

    if(!ros::param::get("/obstacle_avoidance_test/v_max", v_max__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

    if(!ros::param::get("/obstacle_avoidance_test/fly_trajectory_time_out", g_fly_trajectory_time_out)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

}

geometry_msgs::Point get_start(Drone& drone) {
    geometry_msgs::Point start;

    // Get current position from drone
    auto drone_pos = drone.position();
    start.x = drone_pos.x; start.y = drone_pos.y; start.z = drone_pos.z;
    std::cout << "Current position is " << drone_pos.x << " " << drone_pos.y << " " << drone_pos.z << std::endl;

    return start;
}

geometry_msgs::Point get_goal() {
    geometry_msgs::Point goal;

    // Get intended destination from user
    std::cout << "Please input your destination in x,y,z format." << std::endl;
    double input_x, input_y, input_z;
    std::cin >> input_x >> input_y >> input_z;
    goal.x = input_x; goal.y = input_y; goal.z = input_z;

    return goal;
}

trajectory_t request_trajectory(ros::ServiceClient& client, geometry_msgs::Point start, geometry_msgs::Point goal) {
    // Request the actual trajectory from the motion_planner node
    package_delivery::get_trajectory srv;
    srv.request.start = start;
    srv.request.goal = goal;
    int fail_ctr = 0;
    while(!client.call(srv) && fail_ctr<=5){
        fail_ctr++;
    }
     
    if (fail_ctr ==5) {
        ROS_ERROR("Failed to call service.");
        return trajectory_t();

    }
    /* 
    if (client.call(srv)) {
        ROS_INFO("Received trajectory.");
    } else {
        ROS_ERROR("Failed to call service.");
        return trajectory_t();
    }
    */
    return create_trajectory(srv.response.multiDOFtrajectory, true);
}

bool trajectory_done(const trajectory_t& trajectory) {
    return trajectory.size() == 0;
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "obstacle_avoidance_test", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
    ns = ros::this_node::getName();

    //ROS_INFO("HEy!!");
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    obstacle_avoidance_test_initialize_params();
    geometry_msgs::Point start, goal;

    // Flight queues
    trajectory_t normal_traj, rev_normal_traj;
    trajectory_t panic_traj;
    trajectory_t slam_loss_traj;
    trajectory_t future_col_traj;

    bool created_future_col_traj = false;
    bool created_slam_loss_traj = false;

    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin
    
    ros::Time start_hook_t, end_hook_t;                                          
    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        nh.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv", true);
	ros::ServiceClient record_profiling_data_client = 
        nh.serviceClient<profile_manager::profiling_data_srv>("record_profiling_data");
    ros::Subscriber panic_sub = 
		nh.subscribe<std_msgs::Bool>("panic_topic", 1, panic_callback);
    ros::Subscriber panic_velocity_sub = 
		nh.subscribe<geometry_msgs::Vector3>("panic_velocity", 1, panic_velocity_callback);
    ros::Subscriber col_imminent_sub = 
		nh.subscribe<std_msgs::Bool>("col_imminent", 1, col_imminent_callback);
    ros::Subscriber col_coming_sub = 
		nh.subscribe<std_msgs::Bool>("col_coming", 1, col_coming_callback);
	ros::Subscriber slam_lost_sub = 
		nh.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);

    ros::ServiceClient start_profiling_client = 
      nh.serviceClient<profile_manager::start_profiling_srv>("/start_profiling");

    profile_manager::start_profiling_srv start_profiling_srv_inst;
    start_profiling_srv_inst.request.key = "";

    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
    const float goal_s_error_margin = 3.0; //ok distance to be away from the goal.
                                           //this is b/c it's very hard 
                                           //given the issues associated with
                                           //flight controler to land exactly
                                           //on the goal
    ros::Time panic_realization_start_t;
    ros::Time panic_realization_end_t;
    msr::airlib::FlightStats init_stats, end_stats;
    std::string mission_status;
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    
    // Wait for the localization method to come online
    waitForLocalization(localization_method);
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    ros::Time loop_start_t(0,0); 
    ros::Time loop_end_t(0,0); //if zero, it's not valid


    control_drone(drone);
    loop_start_t = ros::Time::now();
    for (State state = setup; ros::ok(); ) {
        if((loop_end_t - loop_start_t).toSec() > g_hand_off_control_time) {  
            control_drone(drone);
            loop_start_t = ros::Time::now();
        }     
        ros::spinOnce();
        
        //testing panic
        trajectory_t * forward_traj = nullptr;
        trajectory_t * rev_traj = nullptr;
        bool check_position = true;
        yaw_strategy_t yaw_strategy = follow_yaw;
        if (should_panic) {
            ROS_ERROR("Panicking!");
            panic_realization_start_t = ros::Time::now();
            panic_traj = create_panic_trajectory(drone, panic_velocity);
            normal_traj.clear(); // Replan a path once we're done
        } else {
            panic_traj.clear();
        }

        if (!panic_traj.empty()) {
            forward_traj = &panic_traj;
            rev_traj = nullptr;
            check_position = false;
            yaw_strategy = ignore_yaw;
            follow_trajectory(drone, forward_traj, rev_traj, yaw_strategy, check_position,v_max__global, g_fly_trajectory_time_out);

        } 

    
        loop_end_t = ros::Time::now();
    
    }
    return 0;
}

