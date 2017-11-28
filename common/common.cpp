#include "common.h"

#include <iostream>
#include <string>
#include <exception>
#include <cmath>

#include <ros/topic.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
#include "Drone.h"


static const int angular_vel = 15;

void update_stats_file(std::string  stats_file__addr, std::string content){
    std::ofstream myfile;
    myfile.open(stats_file__addr);
    myfile<<content<<std::endl;
    myfile.close();
    return;
}
void sigIntHandler(int sig)
{
    ros::shutdown();
    //exit(0);
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

void action_upon_slam_loss(Drone& drone, int level) {
    const std::string lost_topic = "/slam_lost";
    float init_yaw = drone.get_yaw();

    // Stop the drone
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

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
        std_msgs::Bool is_lost = last_msg<std_msgs::Bool>(lost_topic);

        if (!is_lost.data)
            break;
    }
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

