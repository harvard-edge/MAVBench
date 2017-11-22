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
string ip_addr__global;
string localization_method;

enum State { setup, waiting, flying, completed, invalid };
typedef trajectory_msgs::MultiDOFJointTrajectoryPoint multiDOFpoint;
typedef std::deque<multiDOFpoint> trajectory_t;

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
    future_col = msg->data;
}

void package_delivery_initialize_params() {
    ros::param::get("/package_delivery/ip_addr",ip_addr__global);
    ros::param::get("/package_delivery/localization_method",localization_method);
}

// Follows trajectory, popping commands off the front of it and returning those commands in reverse order
trajectory_t follow_trajectory(Drone& drone, trajectory_t& trajectory, float time = 0.5) {
    trajectory_t completed_commands;

    while (time > 0 && trajectory.size() > 1) {
        multiDOFpoint p = trajectory.front();
        multiDOFpoint p_next = trajectory[1];

        // Calculate the positions we should be at
        double p_z = p.transforms[0].translation.z;

        // Calculate the velocities we should be flying at
        double v_x = p.velocities[0].linear.x;
        double v_y = p.velocities[0].linear.y;
        double v_z = p.velocities[0].linear.z;

        // Calculate the time for which these flight commands should run
        double segment_length = (p_next.time_from_start - p.time_from_start).toSec();
        double flight_time = segment_length <= time ? segment_length : time;

        // Fly for flight_time seconds
        auto segment_start_time = std::chrono::system_clock::now();

        drone.fly_velocity(v_x,
                v_y,
                v_z + 0.2*(p_z-drone.position().z));

        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>(flight_time));

        // Push completed command onto stack
        completed_commands.push_back(p);

        // Update trajectory
        trajectory.front().time_from_start += ros::Duration(flight_time);
        if (trajectory.front().time_from_start >= p_next.time_from_start)
            trajectory.pop_front();

        time -= flight_time;
    }

    return completed_commands;
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
    for (multiDOFpoint p : srv.response.multiDOFtrajectory.points)
        result.push_back(p);

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
    ros::NodeHandle n;
    ros::NodeHandle panic_nh;
    signal(SIGINT, sigIntHandler);
	
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    package_delivery_initialize_params();
    trajectory_t trajectory, past_trajectory;
    geometry_msgs::Point start, goal;
	
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    int reaction_delay_counter_init_value = 3;
    int reaction_delay_counter =  reaction_delay_counter_init_value;
    bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin
    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        n.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv");
    ros::Subscriber panic_sub =  
		panic_nh.subscribe<std_msgs::Bool>("panic_topic", 1000, panic_call_back);
    ros::NodeHandle future_col_nh;
    ros::Subscriber future_col_sub = 
		future_col_nh.subscribe<std_msgs::Bool>("future_col_topic", 1000, future_col_callback);

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

        if (state == setup) {
            control_drone(drone);
            goal = get_goal();
            spin_around(drone);
            next_state = waiting;
        }
        else if (state == waiting) {
            start = get_start(drone);
            trajectory = request_trajectory(get_trajectory_client, start, goal);
            next_state = flying;
        }
        else if (state == flying) {
            if (future_col)
                reaction_delay_counter--;

            if (should_panic) {
                action_upon_panic(drone);
                next_state = waiting;
            } else if (future_col && reaction_delay_counter <= 0) {
                action_upon_future_col(drone);
                reaction_delay_counter = reaction_delay_counter_init_value;
                next_state = waiting;
            } else {
                trajectory_t completed_traj = follow_trajectory(drone, trajectory);
                past_trajectory.insert(past_trajectory.end(),
                        completed_traj.begin(), completed_traj.end());

                next_state = trajectory_done(trajectory) ? completed : flying;
            }
        }
        else if (state == completed) {
            if (dist(drone.position(), goal) < goal_s_error_margin) {
                ROS_INFO("Delivered the package and returned!");
                next_state = setup;
            } else { // If we've drifted too far off from the destionation
                start = get_start(drone);
                next_state = waiting;
            }
        }
        else {
            ROS_ERROR("Invalid FSM state!");
            break;
        }

        state = next_state;
        loop_rate.sleep();
    }

    return 0;
}

