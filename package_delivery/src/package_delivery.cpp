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

geometry_msgs::Vector3 panic_direction;
string ip_addr__global;
string localization_method;
string stats_file_addr;
string ns;
std::string g_supervisor_mailbox; //file to write to when completed

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
    
    profiling_data_srv_inst.request.key = "panic_response_time_in_package_delivery";
    profiling_data_srv_inst.request.value = (g_panic_rlzd_t_accumulate/ (double)g_panic_ctr)*1e-9;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "planning_including_ros_overhead_avg";
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

void panic_dir_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    panic_direction = *msg;
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

void package_delivery_initialize_params() {
    if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
      ROS_FATAL_STREAM("Could not start mapping supervisor_mailbox not provided");
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
    if(!ros::param::get("/stats_file_addr",stats_file_addr)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

    if(!ros::param::get("/package_delivery/v_max", v_max__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

    if(!ros::param::get("/package_delivery/fly_trajectory_time_out", g_fly_trajectory_time_out)){
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
    ros::init(argc, argv, "package_delivery", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
    ns = ros::this_node::getName();

    ROS_INFO("HEy!!");
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    package_delivery_initialize_params();
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
    ros::Subscriber panic_dir_sub = 
		nh.subscribe<geometry_msgs::Vector3>("panic_direction", 1, panic_dir_callback);
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
    ROS_INFO_STREAM("HElLO)000000000000!! " << localization_method);
    waitForLocalization(localization_method);
    ROS_INFO("BFDSFSDSDF!!");

    //update_stats_file(stats_file_addr,"\n\n# NEW\n# Package delivery\n###\nTime: ");
    //log_time(stats_file_addr);
    //update_stats_file(stats_file_addr,"###\n");
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    ros::Time loop_start_t(0,0); 
    ros::Time loop_end_t(0,0); //if zero, it's not valid


    for (State state = setup; ros::ok(); ) {
          
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
            ROS_INFO("Waiting to receive trajectory...");

            start = get_start(drone);
            
            start_hook_t = ros::Time::now(); 
            normal_traj = request_trajectory(get_trajectory_client, start, goal);
            end_hook_t = ros::Time::now(); 
            // Pause a little bit so that future_col can be updated
            col_coming = col_imminent = false;
            g_planning_time_including_ros_overhead_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
            g_planning_ctr++; 
            std::this_thread::sleep_for(std::chrono::milliseconds(150));

            if (!normal_traj.empty())
                next_state = flying;
            else
                next_state = failed;
        }
        else if (state == flying)
        {
            trajectory_t * forward_traj = nullptr;
            trajectory_t * rev_traj = nullptr;
            bool check_position = true;
            yaw_strategy_t yaw_strategy = follow_yaw;

            // Handle panic queue
            if (should_panic) {
                ROS_ERROR("Panicking!");
                panic_realization_start_t = ros::Time::now();
                panic_traj = create_panic_trajectory(drone, panic_direction);
                normal_traj.clear(); // Replan a path once we're done
            } else {
                panic_traj.clear();
            }

            // Handle SLAM loss queue
            if (slam_lost) {
                ROS_WARN("SLAM lost!");
                if (!created_slam_loss_traj)
                    slam_loss_traj = create_slam_loss_trajectory(drone, normal_traj, rev_normal_traj);

                // No need to keep the future collision trajectory if we're
                // planning to replan anyway. Keeping it could cause collisions
                // in some cases
                future_col_traj.clear(); 

                created_slam_loss_traj = true;
            } else {
                slam_loss_traj.clear();
                created_slam_loss_traj = false;
            }

            // Handle future_collision queue
            if (col_coming) {
                ROS_WARN("Future collision appeared on trajectory!");

                if (!created_future_col_traj)
                    future_col_traj = create_future_col_trajectory(normal_traj, 1);

                created_future_col_traj = true;

                ROS_WARN_STREAM("Future col length " << future_col_traj.size());

                normal_traj.clear(); // Replan the normal path once we're done
            } else {
                future_col_traj.clear();
                created_future_col_traj = false;
            }

            // Choose correct queue to use
            if (!panic_traj.empty()) {
                forward_traj = &panic_traj;
                rev_traj = nullptr;
                check_position = false;
                yaw_strategy = ignore_yaw;
            } else if (!slam_loss_traj.empty()) {
                forward_traj = &slam_loss_traj;
                rev_traj = &normal_traj;
                check_position = false;
            } else if (!future_col_traj.empty()) {
                forward_traj = &future_col_traj;
                rev_traj = &rev_normal_traj;
            } else {
                forward_traj = &normal_traj;
                rev_traj = &rev_normal_traj;
            }

            if (should_panic){ //CLCT_DATA
                panic_realization_end_t = ros::Time::now();
                g_panic_rlzd_t_accumulate += 
                    (panic_realization_end_t - panic_realization_start_t).toSec()*1e9;
                g_panic_ctr++;
            }
            
            follow_trajectory(drone, forward_traj, rev_traj, yaw_strategy, check_position,v_max__global, g_fly_trajectory_time_out);

            // Choose next state (failure, completion, or more flying)
            if (slam_lost && created_slam_loss_traj && trajectory_done(slam_loss_traj)) {
                if (reset_slam(drone, "/slam_lost"))
                    next_state = completed;
                else
                    next_state = failed;
            }
            else if (trajectory_done(*forward_traj))
                next_state = completed;
            else
                next_state = flying;
        }
        else if (state == completed)
        {
            drone.fly_velocity(0, 0, 0);

            if (dist(drone.position(), goal) < goal_s_error_margin) {
                ROS_INFO("Delivered the package and returned!");
                mission_status = "completed"; 
                g_mission_status = mission_status;            
                //update_stats_file(stats_file_addr,"mission_status completed");
                next_state = setup;
                log_data_before_shutting_down();
                signal_supervisor(g_supervisor_mailbox, "kill"); 
                ros::shutdown();
            } else { //If we've drifted too far off from the destination
                ROS_WARN("We're a little off...");

                auto pos = drone.position();
                std::cout << "Pos: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
                std::cout << "Goal: " << goal.x << " " << goal.y << " " << goal.z << std::endl;
                std::cout << "Dist: " << dist(pos, goal) << std::endl;

                start = get_start(drone);
                next_state = waiting;
            }
        }
        else if (state == failed) {
            ROS_ERROR("Failed to reach destination");
            mission_status = "failed"; 
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

