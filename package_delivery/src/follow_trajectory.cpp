#include "follow_trajectory.h"
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "common_with_external.h"
//#include "template_library.hpp"
#include <sstream>
//#include "api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
//#include "controllers/DroneControllerBase.hpp"
#include "common/Common.hpp"
#include <fstream>
#include "Drone.h"
#include "package_delivery/get_trajectory.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>
#include "common.h"
extern bool should_panic;
extern bool future_col;

void follow_trajecotry(package_delivery::get_trajectory get_trajectory_srv, Drone &drone) {
    int reaction_delay_counter_init_value = 8; 
    int reaction_delay_counter =  reaction_delay_counter_init_value;
    for (int i = 0; !should_panic && i <get_trajectory_srv.response.multiDOFtrajectory.points.size()-1; ++i) {
        auto p = get_trajectory_srv.response.multiDOFtrajectory.points[i];
        auto p_next = get_trajectory_srv.response.multiDOFtrajectory.points[i+1];

        //double p_z = p.transforms[0].translation.z;
        /* 
           double p_x = p.transforms[0].translation.y;
           double p_y = p.transforms[0].translation.x;

           double p_x_next = p_next.transforms[0].translation.y;
           double p_y_next = p_next.transforms[0].translation.x;
           double p_z_next = -p_next.transforms[0].translation.z;
           */
        double v_x = p.velocities[0].linear.x;
        double v_y = p.velocities[0].linear.y;
        double v_z = p.velocities[0].linear.z;

        /* 
           double v_x_next = p_next.velocities[0].linear.y;
           double v_y_next = p_next.velocities[0].linear.x;
           double v_z_next = -p_next.velocities[0].linear.z;
           */

        double segment_dedicated_time = (p_next.time_from_start - p.time_from_start).toSec();
        auto segment_start_time = std::chrono::system_clock::now();

        drone.fly_velocity(v_x, 
                v_y,
                v_z); 
                //v_z + 0.2*(p_z-drone.gps().z));

        ros::spinOnce(); // Check whether we should panic
        /*
        if (should_panic) { //if deteceted a panic
            ROS_ERROR("you should panic\n");
            action_upon_panic(drone);
            break;
        }

        if (future_col){ //if detect a future colision, replan(but wait a bit)
            if(reaction_delay_counter <= 0) {
                ROS_WARN("Obstacle appeared on trajectory");
                action_upon_future_col(drone);
                reaction_delay_counter = reaction_delay_counter_init_value; 
                reaction_delay_counter = reaction_delay_counter_init_value; 
                break;
            }
            reaction_delay_counter--;
        }
        */
        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>((1)*segment_dedicated_time));
    }
}
