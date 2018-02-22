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
geometry_msgs::Vector3 panic_direction;
string ip_addr__global;
string localization_method;
string stats_file_addr;
string ns;
	
enum State { setup, waiting, flying, completed, failed, invalid };

void log_data_before_shutting_down(){
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    profiling_data_srv_inst.request.key = "mission_status";
    profiling_data_srv_inst.request.value = (g_mission_status == "completed" ? 1.0: 0.0);
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

    if (client.call(srv)) {
        ROS_INFO("Received trajectory.");
    } else {
        ROS_ERROR("Failed to call service.");
        return trajectory_t();
    }

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
    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        nh.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv");
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

    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
    const float goal_s_error_margin = 3.0; //ok distance to be away from the goal.
                                           //this is b/c it's very hard 
                                           //given the issues associated with
                                           //flight controler to land exactly
                                           //on the goal

    msr::airlib::FlightStats init_stats, end_stats;
    std::string mission_status;
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    
    // Wait for the localization method to come online
    waitForLocalization(localization_method);

    //update_stats_file(stats_file_addr,"\n\n# NEW\n# Package delivery\n###\nTime: ");
    //log_time(stats_file_addr);
    //update_stats_file(stats_file_addr,"###\n");
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    for (State state = setup; ros::ok(); ) {
        ros::spinOnce();
        State next_state = invalid;

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
            normal_traj = request_trajectory(get_trajectory_client, start, goal);

            // Pause a little bit so that future_col can be updated
            col_coming = col_imminent = false;
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
                    future_col_traj = create_future_col_trajectory(normal_traj, 3);

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

            follow_trajectory(drone, forward_traj, rev_traj, yaw_strategy, check_position);

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
    }
    
    //collect data before shutting down
        //end_stats = drone.getFlightStats();
    //output_flight_summary(init_stats, end_stats, mission_status, stats_file_addr);
    return 0;
}

