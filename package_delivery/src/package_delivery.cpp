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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include "common.h"
#include "timer.h"
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/future_collision.h>

using namespace std;

// Profiling
//global variable to log in stats manager
std::string g_mission_status = "time_out";
ros::Time col_coming_time_stamp; 
long long g_pt_cld_to_pkg_delivery_commun_acc = 0;
int g_col_com_ctr = 0;

bool slam_lost = false;
bool col_coming = false;
bool clcted_col_coming_data = true;
mavbench_msgs::multiDOFtrajectory normal_traj_msg;
mavbench_msgs::multiDOFtrajectory g_next_steps_msg;

long long g_accumulate_loop_time = 0; //it is in ms
long long g_panic_rlzd_t_accumulate = 0;
int g_main_loop_ctr = 0;
int g_panic_ctr = 0;
bool g_start_profiling = false; 

double v_max__global, a_max__global, g_fly_trajectory_time_out;
float g_max_yaw_rate;
float g_max_yaw_rate_during_flight;
double g_planning_budget;
long long g_planning_time_including_ros_overhead_acc  = 0;
int  g_planning_ctr = 0; 
bool clct_data = true;
ros::Time g_traj_time_stamp;

geometry_msgs::Vector3 panic_velocity;
string ip_addr__global;
string localization_method;
string stats_file_addr;
string ns;
std::string g_supervisor_mailbox; //file to write to when completed
bool CLCT_DATA;
bool DEBUG;

enum State { setup, waiting, flying, trajectory_completed, failed, invalid };

void col_coming_callback(const mavbench_msgs::future_collision::ConstPtr& msg)
{
    static int future_col_seq_id = 0;

    if (msg->future_collision_seq < future_col_seq_id)
        return;
    else
        future_col_seq_id = msg->future_collision_seq;

    col_coming = msg->collision;

    if (CLCT_DATA) {
        col_coming_time_stamp = msg->header.stamp;
        g_pt_cld_to_pkg_delivery_commun_acc += (ros::Time::now() - msg->header.stamp).toSec()*1e9;
        g_col_com_ctr++;
    }
}

void get_start_in_future(geometry_msgs::Point& start,
        geometry_msgs::Twist& twist, geometry_msgs::Twist& acceleration)
{
    // Find the conditions the drone will be in when a new path is calculated
    multiDOFpoint mdofp = trajectory_at_time(g_next_steps_msg, g_planning_budget);

    start.x = mdofp.x; start.y = mdofp.y; start.z = mdofp.z; 

    twist.linear.x = mdofp.vx;
    twist.linear.y = mdofp.vy;
    twist.linear.z = mdofp.vz;

    acceleration.linear.x = mdofp.ax;
    acceleration.linear.y = mdofp.ay;
    acceleration.linear.z = mdofp.az;
}

void next_steps_callback(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg)
{
    g_next_steps_msg = *msg;
}

void log_data_before_shutting_down()
{
    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    profiling_data_srv_inst.request.key = "mission_status";
    if (g_mission_status == "time_out") {
        profiling_data_srv_inst.request.value = 0;
    }else if (g_mission_status == "completed") {
        profiling_data_srv_inst.request.value = 1;
    }else {
        profiling_data_srv_inst.request.value = 2;
    }

    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "img_to_pkgDel_commun_t";
    profiling_data_srv_inst.request.value = (((double)g_pt_cld_to_pkg_delivery_commun_acc)/1e9)/g_col_com_ctr;
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

    profiling_data_srv_inst.request.key = "package_delivery_main_loop";
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
    
    profiling_data_srv_inst.request.key = "panic_time_in_package_delivery_node";
    profiling_data_srv_inst.request.value = (g_panic_rlzd_t_accumulate/ (double)g_panic_ctr)*1e-9;
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

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

void package_delivery_initialize_params() {
    if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
      ROS_FATAL_STREAM("Could not start mapping supervisor_mailbox not provided");
      return ;
    }
   
    if(!ros::param::get("/DEBUG",DEBUG))  {
      ROS_FATAL_STREAM("Could not start pkg delivery DEBUG not provided");
      return ;
    }
    
    if(!ros::param::get("/CLCT_DATA",CLCT_DATA))  {
      ROS_FATAL_STREAM("Could not start pkg delivery CLCT_DATA not provided");
      return ;
    }

    if(!ros::param::get("/package_delivery/ip_addr",ip_addr__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/ip_addr").c_str());
      return; 
    }
    if(!ros::param::get("/package_delivery/localization_method",localization_method)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/localization_method").c_str());
       return; 
    }
    if(!ros::param::get("/stats_file_addr",stats_file_addr)) {
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

    if(!ros::param::get("/package_delivery/v_max", v_max__global)) {
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

    if(!ros::param::get("max_yaw_rate",g_max_yaw_rate)) {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate not provided");
        return;
    }

    if(!ros::param::get("max_yaw_rate_during_flight", g_max_yaw_rate_during_flight)) {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate_during_flight not provided");
        return;
    }

    if(!ros::param::get("/package_delivery/fly_trajectory_time_out", g_fly_trajectory_time_out)) {
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
        return; 
    }

    if(!ros::param::get("/planning_budget", g_planning_budget)) {
        ROS_FATAL_STREAM("Could not start pkg delivery planning_budget not provided");
        return;
    }
}

geometry_msgs::Point get_start(Drone& drone) {
    geometry_msgs::Point start;

    // Get current position from drone
    auto drone_pos = drone.position();
    start.x = drone_pos.x; start.y = drone_pos.y; start.z = drone_pos.z;

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

mavbench_msgs::multiDOFtrajectory request_trajectory(ros::ServiceClient& client, const geometry_msgs::Point& start, const geometry_msgs::Point& goal, const geometry_msgs::Twist& twist, const geometry_msgs::Twist& acceleration) {
    // Request the actual trajectory from the motion_planner node
    package_delivery::get_trajectory srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.twist = twist; 
    srv.request.acceleration= acceleration; 
    int fail_ctr = 0;
    
    while(true) {
       if(client.call(srv)) {
           if(!srv.response.path_found) {
               // Back up slowly from current position
               mavbench_msgs::multiDOFtrajectory result;
               result.append = false;
               result.reverse = true;

               return result;
           } else {
               break;
           }
       } else {
           ROS_ERROR("Failed to call service.");
           //return trajectory_t();
       }
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return srv.response.multiDOFtrajectory;
}

bool trajectory_done(const trajectory_t& trajectory) {
    return trajectory.size() == 0;
}

bool drone_stopped()
{
    return g_next_steps_msg.points.size() == 0 &&
        g_next_steps_msg.trajectory_seq >= normal_traj_msg.trajectory_seq;
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "package_delivery", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
    ns = ros::this_node::getName();

    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    package_delivery_initialize_params();
    geometry_msgs::Point start, goal;

    geometry_msgs::Twist twist, acceleration;
    twist.linear.x = twist.linear.y = twist.linear.z = 0;
    acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0; 

    // Flight queues
    trajectory_t slam_loss_traj;
    bool created_slam_loss_traj = false;

    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight);
    bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin
    
    int fail_ctr = 0;
    int fail_threshold = 50;

    ros::Time start_hook_t, end_hook_t;                                          
    // *** F:DN subscribers,publishers,servers,clients
    ros::Publisher trajectory_pub = nh.advertise<mavbench_msgs::multiDOFtrajectory>("normal_traj", 1);

    ros::Subscriber col_coming_sub = nh.subscribe("col_coming", 1, col_coming_callback);
    ros::Subscriber next_steps_sub = nh.subscribe("next_steps", 1, next_steps_callback);
    ros::Subscriber slam_lost_sub = nh.subscribe("/slam_lost", 1, slam_loss_callback);

	ros::ServiceClient get_trajectory_client = 
        nh.serviceClient<package_delivery::get_trajectory>("/get_trajectory_srv");
	ros::ServiceClient record_profiling_data_client = 
        nh.serviceClient<profile_manager::profiling_data_srv>("/record_profiling_data");
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
    
    bool srv_call_status = false;
    ros::Time panic_realization_start_t;
    ros::Time panic_realization_end_t;
    msr::airlib::FlightStats init_stats, end_stats;
    std::string mission_status = "time_out";
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    
    // Wait for the localization method to come online
    waitForLocalization("ground_truth");

    //update_stats_file(stats_file_addr,"\n\n# NEW\n# Package delivery\n###\nTime: ");
    //log_time(stats_file_addr);
    //update_stats_file(stats_file_addr,"###\n");
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    ros::Time loop_start_t(0,0); 
    ros::Time loop_end_t(0,0); //if zero, it's not valid

	ros::Rate pub_rate(80);
    for (State state = setup; ros::ok(); ) {
		pub_rate.sleep();
        ros::spinOnce();
        
        State next_state = invalid;
        loop_start_t = ros::Time::now();
        if (state == setup)
        {
            control_drone(drone);

            goal = get_goal();
            start = get_start(drone);
            
            profiling_data_srv_inst.request.key = "start_profiling";
            if (ros::service::waitForService("/record_profiling_data", 10)){ 
                if(!record_profiling_data_client.call(profiling_data_srv_inst)){
                    ROS_ERROR_STREAM("could not probe data using stats manager");
                    ros::shutdown();
                }
            }
            
            spin_around(drone);
            next_state = waiting;
        }
        else if (state == waiting)
        {
            if (CLCT_DATA)
                start_hook_t = ros::Time::now(); 
            
            normal_traj_msg = request_trajectory(get_trajectory_client, start,
                    goal, twist, acceleration);
           
            // Profiling
            if (CLCT_DATA){ 
                end_hook_t = ros::Time::now(); 

                if (DEBUG)
                    ROS_INFO_STREAM("req traj with srv overhead"<<end_hook_t - start_hook_t); 

                g_planning_time_including_ros_overhead_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
                g_planning_ctr++; 
            } 

            if (!clcted_col_coming_data) { 
                normal_traj_msg.header.stamp = col_coming_time_stamp;
                clcted_col_coming_data = true;
            } else {
                ros::Time temp(0,0); 
                normal_traj_msg.header.stamp = temp;
            }

            trajectory_pub.publish(normal_traj_msg);

            if (!normal_traj_msg.points.empty())
                next_state = flying;
            else
                next_state = trajectory_completed;
        }
        else if (state == flying)
        {
            if (drone_stopped()) {
                next_state = trajectory_completed;

                twist.linear.x = twist.linear.y = twist.linear.z = 0;
                acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
            }
            else if (col_coming) {
                next_state = waiting;
                col_coming = false;
                clcted_col_coming_data = false;

                get_start_in_future(start, twist, acceleration);
            }
            else {
                next_state = flying;
            }
        }
        else if (state == trajectory_completed)
        {
            fail_ctr = normal_traj_msg.points.empty() ? fail_ctr+1 : 0;
            
            if (normal_traj_msg.points.empty()){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (fail_ctr >fail_threshold) {
                next_state = failed;
                mission_status = "planning_failed_too_many_times";
            } else if (dist(drone.position(), goal) < goal_s_error_margin) {
                ROS_INFO("Delivered the package and returned!");
                mission_status = "completed";
                g_mission_status = mission_status;
                //update_stats_file(stats_file_addr,"mission_status completed");
                next_state = setup;
                log_data_before_shutting_down();
                signal_supervisor(g_supervisor_mailbox, "kill"); 
                ros::shutdown();
            } else {
                // If we're too far off from the destination
                start = get_start(drone);
                next_state = waiting;
            }
        }
        else if (state == failed) {
            ROS_ERROR("Failed to reach destination");
            //mission_status = "time_out"; 
            g_mission_status = mission_status;            
            log_data_before_shutting_down();
            signal_supervisor(g_supervisor_mailbox, "kill"); 
            ros::shutdown();
            //update_stats_file(stats_file_addr,"mission_status failed");
            next_state = setup;
        }
        else
        {
            ROS_ERROR("Invalid FSM state!");
            break;
        }

        state = next_state;
        
        if (clct_data){
            if(!g_start_profiling) { 
                if (ros::service::waitForService("/start_profiling", 10)){ 
                    if(!start_profiling_client.call(start_profiling_srv_inst)){
                        ROS_ERROR_STREAM("could not probe data using stats manager");
                        ros::shutdown();
                    }
                    //ROS_INFO_STREAM("now it is true");
                    g_start_profiling = start_profiling_srv_inst.response.start; 
                }
            }
            else{
                //ROS_INFO_STREAM("blah");
                loop_end_t = ros::Time::now(); 
                g_accumulate_loop_time += (((loop_end_t - loop_start_t).toSec())*1e9);
                g_main_loop_ctr++;
            }
        }
    
    }
    //collect data before shutting down
    //end_stats = drone.getFlightStats();
    //output_flight_summary(init_stats, end_stats, mission_status, stats_file_addr);
    return 0;
}

