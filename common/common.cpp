#include "common.h"

#include <iostream>
#include <fstream>
#include <string>
#include <deque>
#include <cmath>
#include <cstdarg>

#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "Drone.h"

static const int angular_vel = 15;

static bool action_upon_slam_loss_backtrack (Drone& drone, const std::string& topic,
        trajectory_t& traj, trajectory_t& reverse_traj);
static bool action_upon_slam_loss_spin(Drone& drone, const std::string& topic);
static bool action_upon_slam_loss_reset(Drone& drone, const std::string& topic);

static trajectory_t append_trajectory(trajectory_t first, trajectory_t second);
static multiDOFpoint reverse_point(multiDOFpoint mdp);

void sigIntHandler(int sig)
{
    ros::shutdown();
    exit(0);
}

void action_upon_future_col(Drone& drone) {
    scan_around(drone, 30);
}

template <class T>
T last_msg (std::string topic) {
    // Return the last message of a latched topic
    return *(ros::topic::waitForMessage<T>(topic));
}

static bool action_upon_slam_loss_reset(Drone& drone, const std::string& topic) {
    ros::NodeHandle nh;
	ros::ServiceClient reset_client = nh.serviceClient<std_srvs::Trigger>("/slam_reset");
    std_srvs::Trigger srv;

    // Reset the SLAM map
    if (reset_client.call(srv)) {
        ROS_INFO("SLAM resetted succesfully");
    } else {
        ROS_ERROR("Failed to reset SLAM");
    }

    // Move around a little to initialize SLAM
    drone.fly_velocity(-0.5, 0, 0, 2);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    drone.fly_velocity(0.5, 0, 0, 4);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    drone.fly_velocity(-0.5, 0, 0, 2);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std_msgs::Bool is_lost = last_msg<std_msgs::Bool>(topic);
    return !is_lost.data;
}

static bool action_upon_slam_loss_spin(Drone& drone, const std::string& topic) {
    float init_yaw = drone.get_yaw();

    // Spin around until we re-localize
    for (int i = angular_vel; i <= 360; i += angular_vel) {
        // Turn slightly
        int angle = init_yaw + i;

        auto start_turn = std::chrono::system_clock::now();
        drone.set_yaw(angle <= 180 ? angle : angle - 360);

        auto end_turn = start_turn + std::chrono::seconds(1);
        std::this_thread::sleep_until(end_turn);

        // Check whether SLAM is back
        std_msgs::Bool is_lost = last_msg<std_msgs::Bool>(topic);

        if (!is_lost.data)
            return true;
    }

    return false;
}

static bool action_upon_slam_loss_backtrack (Drone& drone, const std::string& topic, trajectory_t& traj, trajectory_t& reverse_traj) {
    const double safe_speed = 0.5;

    while (reverse_traj.size() > 1) {
        follow_trajectory(drone, reverse_traj, traj, safe_speed);

        // Check whether SLAM is back
        std_msgs::Bool is_lost = last_msg<std_msgs::Bool>(topic);
        if (!is_lost.data)
            return true;
    }
    ROS_INFO("done");

    return false;
}

bool action_upon_slam_loss (Drone& drone, slam_recovery_method slm...) {
    va_list args;
    va_start(args, slm);

    const std::string lost_topic = "/slam_lost";
    bool success;

    // Stop the drone
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (slm == spin) {
        success = action_upon_slam_loss_spin(drone, lost_topic);
    } else if (slm == backtrack) {
        trajectory_t& traj = *(va_arg(args, trajectory_t*));
        trajectory_t& reverse_traj = *(va_arg(args, trajectory_t*));
        success = action_upon_slam_loss_backtrack(drone, lost_topic, traj, reverse_traj);
    } else if (slm == reset) {
        success = action_upon_slam_loss_reset(drone, lost_topic);
    }

    va_end(args);

    return success;
}

float distance(float x, float y, float z) {
  return std::sqrt(x*x + y*y + z*z);
}

void scan_around(Drone &drone, int angle) {
    float init_yaw = drone.get_yaw();
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ROS_INFO("Scanning around from %f degrees...", init_yaw);

    if (angle > 90) {
		ROS_INFO("we don't have support for angles greater than 90");
        exit(0);
	}
    
    drone.set_yaw(init_yaw+angle <= 180 ? init_yaw + angle : init_yaw + angle - 360);
    drone.set_yaw(init_yaw, true);
    drone.set_yaw(init_yaw-angle >= -180 ? init_yaw - angle : init_yaw - angle + 360);
    drone.set_yaw(init_yaw, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


void spin_slowly(Drone &drone, int n_pies) {
    float init_yaw = drone.get_yaw();
    float angle = 0;
    
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ROS_INFO("spinning around now... asdf");
    
    float  pie_size = 360.0/n_pies;
    for (int i = 0; i <  n_pies; i++) {
        //if (angle <= 90) {
        drone.set_yaw(init_yaw+angle <= 180 ? init_yaw + angle : (init_yaw + angle) - 360);
        angle +=pie_size;
        //}
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}

void spin_around(Drone &drone) {
    float init_yaw = drone.get_yaw();
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ROS_INFO("Spinning around...");

    for (int i = 0; i <= 360; i += angular_vel) {
        int angle = init_yaw + i;

        auto start_turn = std::chrono::system_clock::now();
        drone.set_yaw(angle <= 180 ? angle : angle - 360);

        auto end_turn = start_turn + std::chrono::seconds(1);
        std::this_thread::sleep_until(end_turn);
    }
}

// Follows trajectory, popping commands off the front of it and returning those commands in reverse order
void follow_trajectory(Drone& drone, trajectory_t& traj,
        trajectory_t& reverse_traj, float max_speed, float time) {

    trajectory_t reversed_commands;

    while (time > 0 && traj.size() > 1) {
        multiDOFpoint p = traj.front();
        multiDOFpoint p_next = traj[1];

        // Calculate the positions we should be at
        double p_z = p.transforms[0].translation.z;

        // Calculate the velocities we should be flying at
        double v_x = p.velocities[0].linear.x;
        double v_y = p.velocities[0].linear.y;
        double v_z = p.velocities[0].linear.z;

        // Make sure we're not going over the maximum speed
        double speed = std::sqrt(v_x*v_x + v_y*v_y + v_z*v_z);
        double scale = 1;
        if (speed > max_speed) {
            scale = max_speed / speed; 

            v_x *= scale;
            v_y *= scale;
            v_z *= scale;
        }

        // Calculate the time for which these flight commands should run
        double segment_length = (p_next.time_from_start - p.time_from_start).toSec();
        double flight_time = segment_length <= time ? segment_length : time;

        // Fly for flight_time seconds
        auto segment_start_time = std::chrono::system_clock::now();

        drone.fly_velocity(v_x,
                v_y,
                // v_z);
                v_z + 0.2*(p_z-drone.position().z),
                1,
                true);

        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>(flight_time / scale));

        // Push completed command onto stack
        reversed_commands.push_front(reverse_point(p));

        // Update trajectory
        traj.front().time_from_start += ros::Duration(flight_time);
        if (traj.front().time_from_start >= p_next.time_from_start)
            traj.pop_front();

        time -= flight_time;
    }

    reverse_traj = append_trajectory(reversed_commands, reverse_traj);
}

static multiDOFpoint reverse_point(multiDOFpoint mdp) {
    multiDOFpoint result = mdp;

    result.time_from_start = -mdp.time_from_start;

    result.velocities[0].linear.x = -mdp.velocities[0].linear.x;
    result.velocities[0].linear.y = -mdp.velocities[0].linear.y;
    result.velocities[0].linear.z = -mdp.velocities[0].linear.z;
    
    return result;
}

static trajectory_t append_trajectory (trajectory_t first, trajectory_t second) {
    if (first.size() == 0)
        return second;

    ros::Duration time_step(0.5);
    if (second.size() > 1) {
        time_step = second[1].time_from_start - second[0].time_from_start;
    } else if (first.size() > 1) {
        time_step = first[1].time_from_start - first[0].time_from_start;
    }

    ros::Duration time_shift = first.back().time_from_start;
    time_shift -= second.front().time_from_start;
    time_shift += time_step;

    for (auto mdp : second) {
        mdp.time_from_start += time_shift;
        first.push_back(mdp);
    }

    return first;
}

