#include "ros/ros.h"

#include <iostream>
#include <signal.h>
#include <string>
#include <sstream>
#include <stats_manager/flight_stats_srv.h>
#include "Drone.h"
#include "common.h"

using namespace std;


string g_ip_addr;
string g_stats_fname;
Drone *g_drone;//ip_addr.c_str(), port);
string g_mission_status = "failed";//ip_addr.c_str(), port);
bool g_end_requested = false;
msr::airlib::FlightStats g_init_stats, g_end_stats;
string g_ns;
uint16_t g_port; 


void initialize_params() {
    if(!ros::param::get("/stats_manager/ip_addr",g_ip_addr)){
        ROS_FATAL_STREAM("Could not start exploration. Parameter missing! Looking for stats_manager/ip_addr");
      return; 
    }
    
    if(!ros::param::get("/stats_file_addr", g_stats_fname)){
        ROS_FATAL("Could not start exploration. Parameter missing! Lookining for stats_manager/stats_file_addr");
      return; 
    }
    g_port = 41451;
    /* 
    if(!ros::param::get("/stats_manager/localization_method",localization_method)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/localization_method").c_str());
       return; 
    }
    if(!ros::param::get("/stats_file_addr",stats_file_addr)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }
    */
}




bool probe_flight_stats_cb(stats_manager::flight_stats_srv::Request &req, stats_manager::flight_stats_srv::Response &res)
{
    
    ROS_ERROR_STREAM("insdie the call back"); 
    if (g_drone == NULL) {
        ROS_ERROR_STREAM("drone object is not initialized");
        return false; 
        //res.acquired = false;
    }

    if(req.key == "clct_init_data"){ 
        g_init_stats = g_drone->getFlightStats();
    } 
    else if (req.key == "mission_status"){
        if (req.value == 0.0) {
            g_mission_status = "failed";
        }else{
            g_mission_status = "completed";
        }
    }
    else {
        ROS_ERROR_STREAM("this key is not defined for flight_stats collections"); 
        return false; 
        //res.acquired = false;
    }
    /* 
    else if(req.phase == "clct_end_data"){ 
        g_end_requested = true; 
        g_end_stats = g_drone->getFlightStats();
        g_mission_status = req.mission_status;
    }
    else {
        ROS_ERROR_STREAM("this phase is not defined for flight_stats collections"); 
        return false; 
        //res.acquired = false;
    }
     
    else if(req.phase == "register_mission_status"){ 
        g_mission_status = req.mission_status;
    }
   */  
    return true; 
    //res.acquired = true;
}



// *** F:DN main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "stats_manager");
    ros::NodeHandle nh;
    ros::ServiceServer probe_flight_stats_service = nh.advertiseService("probe_flight_stats", probe_flight_stats_cb);
    initialize_params();
    g_drone = new Drone(g_ip_addr.c_str(), g_port);
    ros::Rate loop_rate(2);
    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    g_end_stats = g_drone->getFlightStats();
    output_flight_summary(g_init_stats, g_end_stats, g_mission_status, g_stats_fname);
    //output_flight_summary(drone, stats_fname);
    return 0;
}

