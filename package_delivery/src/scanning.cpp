#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "template_library.hpp"
#include <sstream>
//#include "rpc/RpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
#include <tuple>
//#include "controllers/DroneControllerBase.hpp"
#include "control_drone.h"
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

using namespace std;
string ip_addr__global;
string localization_method;
string stats_file_addr;
string ns;

enum State { setup, waiting, flying, completed, invalid };

double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.y)*(t.x-m.y) + (t.y-m.x)*(t.y-m.x) + (t.z+m.z)*(t.z+m.z));
}

void package_delivery_initialize_params() {
    if(!ros::param::get("/scanning/ip_addr",ip_addr__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/ip_addr").c_str());
    }
    if(!ros::param::get("/scanning/localization_method",localization_method)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/localization_method").c_str());
    }

    if(!ros::param::get("/stats_file_addr",stats_file_addr)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
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

void get_goal(int& width, int& length, int& lanes) {
    std::cout << "Enter width ,length and number of lanes"<<std::endl
        << "associated with the area you like to sweep "<<endl;

    std::cin >> width >> length >> lanes;
}

trajectory_t request_trajectory(ros::ServiceClient& client, geometry_msgs::Point start, int width, int length, int lanes) {
    // Request the actual trajectory from the motion_planner node
    package_delivery::get_trajectory srv;
    srv.request.start = start;
    srv.request.width = width;
    srv.request.length = length;
    srv.request.n_pts_per_dir = lanes;

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

bool trajectory_done(trajectory_t trajectory) {
    return trajectory.size() <= 1;
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "scanning", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle panic_nh;
    signal(SIGINT, sigIntHandler);

    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    string app_name;
    package_delivery_initialize_params();

    int width, length, lanes; // size of area to scan
    geometry_msgs::Point start, goal, original_start;

	package_delivery::get_trajectory get_trajectory_srv;
    trajectory_t trajectory, reverse_trajectory;
	
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    
    //bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin

    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        n.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv");
    
    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
    //const int step__total_number = 1;
    int package_delivery_loop_rate = 100;
    float goal_s_error_margin = 5.0; //ok distance to be away from the goal.
                                                      //this is b/c it's very hard 
                                                      //given the issues associated with
                                                      //flight controler to land exactly
                                                      //on the goal

    
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    ros::Rate loop_rate(package_delivery_loop_rate);
    for (State state = setup; ros::ok(); ) {
        ros::spinOnce();
        State next_state = invalid;
        //std::cout<<stats_file_addr<<std::endl;
        //update_stats_file(stats_file_addr,"now now");

        if (state == setup)
        {
            control_drone(drone);

            get_goal(width, length, lanes);
            original_start = get_start(drone);

            next_state = waiting;
        }
        else if (state == waiting)
        {
            ROS_INFO("Waiting to receive trajectory...");
            start = get_start(drone);
            trajectory = request_trajectory(get_trajectory_client, start, width, length, lanes);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            next_state = flying;
        }
        else if (state == flying)
        {
            follow_trajectory(drone, trajectory, reverse_trajectory);
            next_state = trajectory_done(trajectory) ? completed : flying;
        }
        else if (state == completed)
        {
            drone.fly_velocity(0, 0, 0);
            ROS_INFO("scanned the entire space and returned successfully");
            update_stats_file(stats_file_addr,"mission_status completed");
            next_state = setup;
        }
        else
        {
            ROS_ERROR("Invalid FSM state!");
            break;
        }

        state = next_state;
    }


    return 0;
}

