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
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>

using namespace std;
string ip_addr__global;
string localization_method;
string g_stats_file_addr;
string ns;
std::string g_supervisor_mailbox; //file to write to when completed
std::string g_mission_status = "failed";
long long g_planning_time_acc = 0;
int g_planning_ctr = 0;
long long g_accumulate_loop_time = 0;
int g_main_loop_ctr = 0;

bool g_start_profiling = false; 
double v_max__global, a_max__global, g_fly_trajectory_time_out;
enum State { setup, waiting, flying, completed, invalid };
bool clct_data = true;


double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.y)*(t.x-m.y) + (t.y-m.x)*(t.y-m.x) + (t.z+m.z)*(t.z+m.z));
}


void initialize_params() {
    if(!ros::param::get("/scanning/ip_addr",ip_addr__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/ip_addr").c_str());
        return; 
    }
    if(!ros::param::get("/scanning/localization_method",localization_method)){
        ROS_FATAL("Could not start scanning. Parameter missing! Looking for %s", 
                (ns + "/localization_method").c_str());
        return; 
    }

    if(!ros::param::get("/stats_file_addr",g_stats_file_addr)){
        ROS_FATAL("Could not start scanning . Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
        return;
    }

    if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
        ROS_FATAL_STREAM("Could not start scanning supervisor_mailbox not provided");
        return;
    }

    if(!ros::param::get("/scanning/v_max", v_max__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }
    
    if(!ros::param::get("/scanning/fly_trajectory_time_out", g_fly_trajectory_time_out)){
        ROS_FATAL("Could not start exploration. Parameter missing! fly_trajectory_time_out not provided");
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


void get_goal(int& width, int& length, int& lanes) {
    std::cout << "Enter width ,length and number of lanes"<<std::endl
        << "associated with the area you like to sweep "<<endl;

    std::cin >> width >> length >> lanes;
}


trajectory_t request_trajectory(ros::ServiceClient& client, geometry_msgs::Point start, int width, int length, int lanes) {
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

    return create_trajectory(srv.response.multiDOFtrajectory);
}


bool trajectory_done(trajectory_t trajectory) {
    return trajectory.size() == 0;
}


void log_data_before_shutting_down(){
    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;

    profiling_data_srv_inst.request.key = "mission_status";
    profiling_data_srv_inst.request.value = (g_mission_status == "completed" ? 1.0: 0.0);
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "scanning_main_loop";
    profiling_data_srv_inst.request.value = (((double)g_accumulate_loop_time)/1e9)/g_main_loop_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
        }
    }

    profiling_data_srv_inst.request.key = "planning_time";
    profiling_data_srv_inst.request.value = ((double)g_planning_time_acc/g_planning_ctr)/1e9;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
}

void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        ros::shutdown();
    }
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanning", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    string app_name;
    initialize_params();
    int width, length, lanes; // size of area to scan
    ros::Time start_hook_t, end_hook_t;  
    geometry_msgs::Point start, goal, original_start;
	package_delivery::get_trajectory get_trajectory_srv;
    trajectory_t trajectory;
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    signal(SIGINT, sigIntHandlerPrivate);
	ros::ServiceClient get_trajectory_client = 
        n.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv");
    ros::ServiceClient record_profiling_data_client = 
      n.serviceClient<profile_manager::profiling_data_srv>("/record_profiling_data");

    ros::ServiceClient start_profiling_client = 
      n.serviceClient<profile_manager::start_profiling_srv>("/start_profiling");

    profile_manager::start_profiling_srv start_profiling_srv_inst;
    start_profiling_srv_inst.request.key = "";
    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
    //const int step__total_number = 1;
    int scanning_loop_rate = 100;
    float goal_s_error_margin = 5.0; //ok distance to be away from the goal.
                                                      //this is b/c it's very hard 
                                                      //given the issues associated with
                                                      //flight controler to land exactly
                                                      //on the goal
    
    ros::Time loop_start_t(0,0); 
    ros::Time loop_end_t(0,0); //if zero, it's not valid
    
    ros::Rate loop_rate(scanning_loop_rate);
    for (State state = setup; ros::ok(); ) {
        ros::spinOnce();
        State next_state = invalid;
        loop_start_t = ros::Time::now();
        
        if (state == setup){
            control_drone(drone);
            get_goal(width, length, lanes);
            original_start = get_start(drone);
            
            profile_manager::profiling_data_srv profiling_data_srv_inst;
            profiling_data_srv_inst.request.key = "start_profiling";
            if (ros::service::waitForService("/record_profiling_data", 10)){ 
                if(!record_profiling_data_client.call(profiling_data_srv_inst)){
                    ROS_ERROR_STREAM("could not probe data using stats manager");
                    ros::shutdown();
                }
            }
            
            next_state = waiting;
        } else if (state == waiting) {
            ROS_INFO("Waiting to receive trajectory...");
            start = get_start(drone);
            start_hook_t = ros::Time::now(); 
            trajectory = request_trajectory(get_trajectory_client, start, width, length, lanes);
            end_hook_t = ros::Time::now(); 
            g_planning_time_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
            g_planning_ctr++; 

            //std::this_thread::sleep_for(std::chrono::seconds(1));
            next_state = flying;
        } else if (state == flying)
        {
            follow_trajectory(drone, &trajectory, nullptr, 
                    ignore_yaw, true ,v_max__global, g_fly_trajectory_time_out);
            //follow_trajectory(drone, &trajectory, &reverse_trajectory, ignore_yaw, true);
            next_state = trajectory_done(trajectory) ? completed : flying;
        } else if (state == completed){
            drone.fly_velocity(0, 0, 0);
            ROS_INFO("scanned the entire space and returned successfully");
            //update_stats_file(stats_file_addr,"mission_status completed");
            g_mission_status = "completed";
            log_data_before_shutting_down();
            signal_supervisor(g_supervisor_mailbox, "kill"); 
            ros::shutdown(); 
            //next_state = setup;
        } else {
            ROS_ERROR("Invalid FSM state!");
            break;
        }

        state = next_state;
    
        if (clct_data){
            if(!g_start_profiling) { 
                if (ros::service::waitForService("/start_profiling", 10)){ 
                    if(!start_profiling_client.call(start_profiling_srv_inst)){
                        ROS_ERROR_STREAM("could not probe data using stats manager");
                        ros::shutdown();
                    }
                    //ROS_INFO_STREAM("now it is true");
                    g_start_profiling = start_profiling_srv_inst.response.start; 
                }
            }
            else{
                //ROS_INFO_STREAM("blah");
                loop_end_t = ros::Time::now(); 
                g_accumulate_loop_time += (((loop_end_t - loop_start_t).toSec())*1e9);
                g_main_loop_ctr++;
            }
        }
    
    }

    return 0;
}
