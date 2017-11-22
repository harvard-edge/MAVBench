#include "common.h"

#include <iostream>
#include <string>
#include <cmath>
#include <deque>

#include <ros/topic.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include "Drone.h"

static const int angular_vel = 15;
typedef trajectory_msgs::MultiDOFJointTrajectoryPoint multiDOFpoint;
typedef std::deque<multiDOFpoint> trajectory_t;

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
    T result;
    result = *(ros::topic::waitForMessage<T>(topic));

    return result;
}

void action_upon_slam_loss_spin(Drone& drone, const std::string& topic) {
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
        bool timed_out;
        std_msgs::Bool is_lost = last_msg<std_msgs::Bool>(topic);

        if (!is_lost.data)
            break;
    }
}

void action_upon_slam_loss_backtrack (Drone& drone, const std::string& topic, trajectory_t& trajectory) {

}

void action_upon_slam_loss (Drone& drone, slam_recovery_method slm...) {
    va_list args;
    va_start(args, slm);

    const std::string lost_topic = "/slam_lost";

    // Stop the drone
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (slm == spin) {
        action_upon_slam_loss_spin(drone, lost_topic);
    } else if (slm == backtrack) {
        trajectory_t& traj = *(va_arg(args, trajectory_t *));
        action_upon_slam_loss_backtrack(drone, lost_topic, traj);
    }

    va_end(args);
}

float distance(float x, float y, float z) {
  return std::sqrt(x*x + y*y + z*z);
}

void scan_around(Drone &drone, int angle) {
    float init_yaw = drone.get_yaw();
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ROS_INFO("Scanning around...");

    if (angle > 90) {
		ROS_INFO("we don't have support for angles greater than 90");
        exit(0);
	}
    
    drone.set_yaw(init_yaw+angle <= 180 ? init_yaw + angle : 360 - init_yaw - angle);
    drone.set_yaw(init_yaw);
    drone.set_yaw(init_yaw-angle <= 180 ? init_yaw - angle : 360 - init_yaw + angle);
    drone.set_yaw(init_yaw);
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
    
    /* 
    if (angle <= 90) {
		drone.set_yaw(init_yaw+angle <= 180 ? init_yaw + angle : 360 - init_yaw - angle);
		drone.set_yaw(init_yaw);
		drone.set_yaw(init_yaw-angle <= 180 ? init_yaw - angle : 360 - init_yaw + angle);
		drone.set_yaw(init_yaw);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }else{
		ROS_INFO("we don't have support for angles greater than 90");
        exit(0);
	}
    */
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

