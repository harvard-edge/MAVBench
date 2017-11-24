#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <iostream>
#include <chrono>
#include <cmath>
#include <chrono>
#include <thread>
#include <deque>
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
#include "follow_trajectory.h"

using namespace std;
bool should_panic = false;
bool future_col = false;
bool slam_lost = false;
string ip_addr__global;
string localization_method;

enum State { setup, waiting, flying, completed, invalid };

double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.x)*(t.x-m.x) + (t.y-m.y)*(t.y-m.y) + (t.z+m.z)*(t.z-m.z));
}

// *** F:DN call back function for the panic_topic subscriber
void panic_call_back(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
}

void future_col_callback(const std_msgs::Bool::ConstPtr& msg) {
    const int reaction_delay_counter_init_value = 3;
    static int reaction_delay_counter = reaction_delay_counter_init_value;

    if (!msg->data) {
    	reaction_delay_counter = reaction_delay_counter_init_value;
    } else {
    	reaction_delay_counter--;
    }

    if (msg->data && reaction_delay_counter <= 0)
    	future_col = true;
    else
    	future_col = false;
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

void package_delivery_initialize_params() {
    ros::param::get("/package_delivery/ip_addr",ip_addr__global);
    ros::param::get("/package_delivery/localization_method",localization_method);
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
        // p.velocities[0].linear.x = -3;
        // p.velocities[0].linear.y = 0;
        // p.velocities[0].linear.z = 0;

        result.push_back(p);
    }

    return result;
}

bool trajectory_done(trajectory_t trajectory) {
    return trajectory.size() <= 1;
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "package_delivery", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
	
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    package_delivery_initialize_params();
    trajectory_t trajectory, reverse_trajectory;
    geometry_msgs::Point start, goal;
	
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin
    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        nh.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv");
    ros::Subscriber panic_sub =  
		nh.subscribe<std_msgs::Bool>("panic_topic", 1000, panic_call_back);
    ros::Subscriber future_col_sub = 
		nh.subscribe<std_msgs::Bool>("future_col_topic", 1000, future_col_callback);
	ros::Subscriber slam_lost_sub = 
		nh.subscribe<std_msgs::Bool>("/slam_lost", 1000, slam_loss_callback);

    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
    const int package_delivery_loop_rate = 50;
    float goal_s_error_margin = 2.0; //ok distance to be away from the goal.
                                                      //this is b/c it's very hard 
                                                      //given the issues associated with
                                                      //flight controler to land exactly
                                                      //on the goal

    
    
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    ros::Rate loop_rate(package_delivery_loop_rate);
    LOG_TIME(package_delivery);
    for (State state = setup; ros::ok(); ) {
        ros::spinOnce();
        State next_state = invalid;

        if (state == setup)
        {
            control_drone(drone);
            goal = get_goal();
            // spin_around(drone);
            next_state = waiting;
        }
        else if (state == waiting)
        {
            ROS_INFO("Waiting to recieve trajectory...");
            start = get_start(drone);
            trajectory = request_trajectory(get_trajectory_client, start, goal);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            next_state = flying;
        }
        else if (state == flying)
        {
            if (should_panic) {
                ROS_WARN("Panic! in the disco");
                action_upon_panic(drone);
                next_state = waiting;
            } else if (future_col) {
                ROS_WARN("Future collision detected on trajectory!");
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

                // if (!slam_found) {
                //     ROS_INFO("Reseting SLAM");
                //     slam_found = action_upon_slam_loss(drone, reset);
                // }

                if (slam_found) {
                    ROS_INFO("Recovered SLAM!");
                    next_state = flying;
                } else {
                    ROS_INFO("SLAM not recovered! Just do it yourself");
                    next_state = setup;
                }
            } else {
                follow_trajectory(drone, trajectory, reverse_trajectory);
                next_state = trajectory_done(trajectory) ? completed : flying;
            }
        }
        else if (state == completed)
        {
            if (dist(drone.position(), goal) < goal_s_error_margin) {
                ROS_INFO("Delivered the package and returned!");
                next_state = setup;
            } else { //If we've drifted too far off from the destionation
                ROS_WARN("We're a little off...");
                start = get_start(drone);
                next_state = waiting;
            }
        }
        else
        {
            ROS_ERROR("Invalid FSM state!");
            break;
        }

        state = next_state;
        loop_rate.sleep();
    }

    return 0;
}

