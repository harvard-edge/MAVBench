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

#include "control_drone.h"
#include "common/Common.hpp"
#include "Drone.h"
#include "package_delivery/get_trajectory.h"
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "common.h"
#include "timer.h"

using namespace std;
bool should_panic = false;
bool col_imminent = false;
bool col_coming = false;
bool slam_lost = false;
string ip_addr__global;
string localization_method;
string stats_file_addr;
string ns;
	
enum State { setup, waiting, flying, completed, failed, invalid };

double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.x)*(t.x-m.x) + (t.y-m.y)*(t.y-m.y) + (t.z-m.z)*(t.z-m.z));
}

// *** F:DN call back function for the panic_topic subscriber
void panic_call_back(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
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

    trajectory_t result;
    for (multiDOFpoint p : srv.response.multiDOFtrajectory.points) {
        result.push_back(p);
    }

    return result;
}

bool trajectory_done(const trajectory_t& trajectory) {
    return trajectory.size() <= 1;
}

double trajectory_start_time(const trajectory_t& trajectory)
{
    if (trajectory.empty())
        return 0;
    return trajectory.front().time_from_start.toSec();
}

void trajectory_shift_time(trajectory_t& trajectory, double shift)
{
    ros::Duration dur = ros::Duration(shift);
    for (auto& mdp : trajectory)
        mdp.time_from_start += dur;
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "package_delivery", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
    ns = ros::this_node::getName();
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    package_delivery_initialize_params();
    geometry_msgs::Point start, goal;
    trajectory_t trajectory, reverse_trajectory;
    double start_time = 0;

    double max_speed = std::numeric_limits<double>::infinity();
    double max_speed_reset_time = 0;
    double max_speed_increment_time = 0;
	
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin
    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        nh.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv");
    ros::Subscriber panic_sub =  
		nh.subscribe<std_msgs::Bool>("panic_topic", 1000, panic_call_back);
    ros::Subscriber col_imminent_sub = 
		nh.subscribe<std_msgs::Bool>("col_imminent", 1000, col_imminent_callback);
    ros::Subscriber col_coming_sub = 
		nh.subscribe<std_msgs::Bool>("col_coming", 1000, col_coming_callback);
	ros::Subscriber slam_lost_sub = 
		nh.subscribe<std_msgs::Bool>("/slam_lost", 1000, slam_loss_callback);

    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
    const double max_safe_speed = 1.0;
    const double max_speed_increment = 1.0;
    const double max_speed_reset_time_length = 4.0;
    const float goal_s_error_margin = 3.0; //ok distance to be away from the goal.
                                           //this is b/c it's very hard 
                                           //given the issues associated with
                                           //flight controler to land exactly
                                           //on the goal

    
    
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    update_stats_file(stats_file_addr,"\n\n# NEW\n# Package delivery\n###\nTime: ");
    log_time(stats_file_addr);
    update_stats_file(stats_file_addr,"###\n");

    for (State state = setup; ros::ok(); ) {
        ros::spinOnce();
        State next_state = invalid;

        if (state == setup)
        {
            control_drone(drone);

            goal = get_goal();
            start = get_start(drone);

            spin_around(drone);
            next_state = waiting;
        }
        else if (state == waiting)
        {
            ROS_INFO("Waiting to receive trajectory...");

            start_time = trajectory_start_time(trajectory);

            start = get_start(drone);
            trajectory = request_trajectory(get_trajectory_client, start, goal);

            trajectory_shift_time(trajectory, start_time);

            // Pause a little bit so that future_col can be updated
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            if (!trajectory.empty())
                next_state = flying;
            else
                next_state = failed;
        }
        else if (state == flying)
        {
            if (should_panic) {
                ROS_WARN("Panic! in the disco");
                action_upon_panic(drone);
                next_state = waiting;
            } else if (col_imminent) {
                ROS_WARN("Reacting to future collision on trajectory");
                action_upon_future_col(drone);
                next_state = waiting;
            } else if (slam_lost) {
                ROS_WARN("SLAM localization lost!");
                bool slam_found = false;

                // ROS_INFO("Spinning to regain SLAM");
            	// slam_found = action_upon_slam_loss(drone, spin);

                if (!slam_found) {
                    ROS_INFO("Backtracking to regain SLAM");
                    slam_found = action_upon_slam_loss(drone, backtrack,
                            &trajectory, &reverse_trajectory);
                }

                if (!slam_found) {
                    ROS_INFO("Reseting SLAM");
                    slam_found = action_upon_slam_loss(drone, reset);
                }

                if (slam_found) {
                    ROS_INFO("Recovered SLAM!");

                    // Slow down until we pass a little beyond the point where
                    // SLAM was lost
                    max_speed = max_safe_speed;
                    max_speed_reset_time = trajectory_start_time(trajectory) + max_speed_reset_time_length;
                    max_speed_increment_time = trajectory_start_time(trajectory) + 1;

                    next_state = flying;
                } else {
                    ROS_WARN("SLAM not recovered! Just do it yourself");
                    next_state = setup;
                }
            // } else if (col_coming) {
            //     follow_trajectory(drone, trajectory, reverse_trajectory, face_forward, max_safe_speed);
            //     next_state = trajectory_done(trajectory) ? completed : flying;
            } else {
                follow_trajectory(drone, trajectory, reverse_trajectory, face_forward, max_speed);
                next_state = trajectory_done(trajectory) ? completed : flying;
            }
        }
        else if (state == completed)
        {
            drone.fly_velocity(0, 0, 0);

            if (dist(drone.position(), goal) < goal_s_error_margin) {
                ROS_INFO("Delivered the package and returned!");
                update_stats_file(stats_file_addr,"mission_status completed");
                next_state = setup;
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
            update_stats_file(stats_file_addr,"mission_status failed");
            next_state = setup;
        }
        else
        {
            ROS_ERROR("Invalid FSM state!");
            break;
        }

        // Update max_speed if required
        double now = trajectory_start_time(trajectory);
        if (now > max_speed_reset_time) {
            max_speed = std::numeric_limits<double>::infinity();
        } else if (now > max_speed_increment_time) {
            max_speed += max_speed_increment;
            max_speed_increment_time += 1;
        }

        state = next_state;
    }

    return 0;
}

