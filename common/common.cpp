#include "common.h"
#include <iostream>
#include <cmath>
#include <random>
#include "Drone.h"



void action_upon_future_col(Drone& drone) {
    scan_around(drone, 30);
}
float distance(float x, float y, float z) {
  return std::sqrt(x*x + y*y + z*z);
}

void scan_around(Drone &drone, int angle) {
    float init_yaw = drone.get_yaw();
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ROS_INFO("Scanning around...");
    
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
    drone.set_yaw(init_yaw+90 <= 180 ? init_yaw + 90 : 360 - init_yaw - 90);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    drone.set_yaw(init_yaw+180 <= 180 ? init_yaw + 180 : 360 - init_yaw - 180);
    drone.set_yaw(init_yaw-90 <= 180 ? init_yaw - 90 : 360 - init_yaw + 90);
    drone.set_yaw(init_yaw);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
