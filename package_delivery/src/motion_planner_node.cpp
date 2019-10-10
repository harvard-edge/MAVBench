#include "ros/ros.h"

// Standard headers
#include <string>
#include <signal.h>

// MAVBench headers
#include "Drone.h"
#include "timer.h"
#include "motion_planner.h"
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>

MotionPlanner * mp_ptr = nullptr;
void sigIntHandlerPrivate(int signo) {
    if (signo == SIGINT) {
        mp_ptr->log_data_before_shutting_down();
        ros::shutdown();
    }
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_planner_node");
    const std::string blah = "~";
    ros::NodeHandle nh(blah);
    signal(SIGINT, sigIntHandlerPrivate);

    ROS_WARN("This node has no OctoMap instantiated! It's only useful for the scanning application");

    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    
    // Create a Drone object
    std::string ip_addr, localization_method;
    ros::param::get("/ip_addr", ip_addr);
    uint16_t port = 41451;
    if(!ros::param::get("/localization_method", localization_method)) {
        ROS_FATAL("Could not start occupancy map node. Localization parameter missing!");
        exit(-1);
    }
    Drone drone(ip_addr.c_str(), port, localization_method);
    
    // Create MotionPlanner
    MotionPlanner mp (nullptr, &drone);
    mp_ptr = &mp;

    while (ros::ok()) {
        ros::spinOnce();
        mp.spinOnce();
    }
}

