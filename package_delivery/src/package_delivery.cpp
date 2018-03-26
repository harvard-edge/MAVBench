#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <iostream>
#include <chrono>
#include <cmath>
#include <chrono>
#include <thread>
#include <deque>
#include <limits>
#include <signal.h>

#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include "control_drone.h"
#include "common/Common.hpp"
#include "Drone.h"
#include "package_delivery/get_trajectory.h"
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include "common.h"
#include "timer.h"
#include <package_delivery/multiDOF.h>
#include <package_delivery/multiDOF_array.h>
#include <package_delivery/follow_trajectory_status_srv.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <package_delivery/BoolPlusHeader.h>

using namespace std;

// Profiling
//global variable to log in stats manager
std::string g_mission_status = "failed";
ros::Time col_coming_time_stamp; 
long long g_pt_cld_to_pkg_delivery_commun_acc = 0;
int g_col_com_ctr = 0;

bool should_panic = false;
bool slam_lost = false;
bool col_coming = false;
bool clcted_col_coming_data = true;


long long g_accumulate_loop_time = 0; //it is in ms
long long g_panic_rlzd_t_accumulate = 0;
int g_main_loop_ctr = 0;
int g_panic_ctr = 0;
bool g_start_profiling = false; 

double v_max__global, a_max__global, g_fly_trajectory_time_out;
float g_max_yaw_rate;
float g_max_yaw_rate_during_flight;
long long g_planning_time_including_ros_overhead_acc  = 0;
int  g_planning_ctr = 0; 
bool clct_data = true;
ros::Time g_traj_time_stamp;



geometry_msgs::Vector3 panic_velocity;
string ip_addr__global;
string localization_method;
string stats_file_addr;
string ns;
std::string g_supervisor_mailbox; //file to write to when completed
bool CLCT_DATA;
bool DEBUG;

enum State { setup, waiting, flying, trajectory_completed, failed, invalid };

void col_coming_callback(const package_delivery::BoolPlusHeader::ConstPtr& msg) {
    col_coming = msg->data;
    if (CLCT_DATA){ 
        col_coming_time_stamp = msg->header.stamp;
        //col_coming_time_stamp = ros::Time::now();
        g_pt_cld_to_pkg_delivery_commun_acc += (ros::Time::now() - msg->header.stamp).toSec()*1e9;
        g_col_com_ctr++;
    }
    //ROS_INFO_STREAM("col_coming to col_coming_cb"<<ros::Time::now() - col_coming_time_stamp);

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
    
    profiling_data_srv_inst.request.key = "img_to_pkgDel_commun_t";
    profiling_data_srv_inst.request.value = (((double)g_pt_cld_to_pkg_delivery_commun_acc)/1e9)/g_col_com_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "motion_planning_plus_srv_call";
    profiling_data_srv_inst.request.value = ((double)g_planning_time_including_ros_overhead_acc/g_planning_ctr)/1e9;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "package_delivery_main_loop";
    profiling_data_srv_inst.request.value = (((double)g_accumulate_loop_time)/1e9)/g_main_loop_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
        }
    }

    profiling_data_srv_inst.request.key = "panic_ctr";
    profiling_data_srv_inst.request.value = g_panic_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "panic_time_in_package_delivery_node";
    profiling_data_srv_inst.request.value = (g_panic_rlzd_t_accumulate/ (double)g_panic_ctr)*1e-9;
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

double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.x)*(t.x-m.x) + (t.y-m.y)*(t.y-m.y) + (t.z-m.z)*(t.z-m.z));
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

void package_delivery_initialize_params() {
    if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
      ROS_FATAL_STREAM("Could not start mapping supervisor_mailbox not provided");
      return ;
    }
   
    if(!ros::param::get("/DEBUG",DEBUG))  {
      ROS_FATAL_STREAM("Could not start pkg delivery DEBUG not provided");
      return ;
    }
    
    if(!ros::param::get("/CLCT_DATA",CLCT_DATA))  {
      ROS_FATAL_STREAM("Could not start pkg delivery CLCT_DATA not provided");
      return ;
    }

    if(!ros::param::get("/package_delivery/ip_addr",ip_addr__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/ip_addr").c_str());
      return; 
    }
    if(!ros::param::get("/package_delivery/localization_method",localization_method)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/localization_method").c_str());
       return; 
    }
    if(!ros::param::get("/stats_file_addr",stats_file_addr)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

    if(!ros::param::get("/package_delivery/v_max", v_max__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

    if(!ros::param::get("max_yaw_rate",g_max_yaw_rate))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate not provided");
        return;
    }

    if(!ros::param::get("max_yaw_rate_during_flight",g_max_yaw_rate_during_flight))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate_during_flight not provided");
        return;
    }

    if(!ros::param::get("/package_delivery/fly_trajectory_time_out", g_fly_trajectory_time_out)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }

}

geometry_msgs::Point get_start(Drone& drone) {
    geometry_msgs::Point start;

    // Get current position from drone
    auto drone_pos = drone.position();
    start.x = drone_pos.x; start.y = drone_pos.y; start.z = drone_pos.z;
    //std::cout << "Current position is " << drone_pos.x << " " << drone_pos.y << " " << drone_pos.z << std::endl;

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

trajectory_t request_trajectory(ros::ServiceClient& client, geometry_msgs::Point start, geometry_msgs::Point goal, geometry_msgs::Twist twist, geometry_msgs::Twist acceleration) {
    // Request the actual trajectory from the motion_planner node
    package_delivery::get_trajectory srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.twist  = twist; 
    srv.request.acceleration= acceleration; 
    int fail_ctr = 0;
    
    /*
    while(!client.call(srv) && fail_ctr<=5){
        fail_ctr++;
    }
     
    if (fail_ctr ==5) {
        ROS_ERROR("Failed to call service.");
        return trajectory_t();

    }
    */ 
   while(true){ 
       if(client.call(srv)) {
           //ROS_INFO("Received trajectory.");
           if(!srv.response.path_found){
               return trajectory_t();
           }else{
               break;
           }
       } else {
           ROS_ERROR("Failed to call service.");
           //return trajectory_t();
       }
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return create_trajectory(srv.response.multiDOFtrajectory, true);
}

bool trajectory_done(const trajectory_t& trajectory) {
    return trajectory.size() == 0;
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "package_delivery", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
    ns = ros::this_node::getName();

    //ROS_INFO("HEy!!");
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    package_delivery_initialize_params();
    geometry_msgs::Point start, goal;

    // Flight queues
    trajectory_t slam_loss_traj;
    bool created_slam_loss_traj = false;

    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight);
    bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin
    
   package_delivery::follow_trajectory_status_srv follow_trajectory_status_srv_inst;

   int fail_ctr = 0;
   int fail_threshold = 15;

    ros::Time start_hook_t, end_hook_t;                                          
    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        nh.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv", true);
	ros::ServiceClient record_profiling_data_client = 
        nh.serviceClient<profile_manager::profiling_data_srv>("record_profiling_data");
        ros::Subscriber slam_lost_sub = nh.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);

    ros::ServiceClient follow_trajectory_status_client = 
      nh.serviceClient<package_delivery::follow_trajectory_status_srv>("/follow_trajectory_status", true);

    ros::Publisher trajectory_pub = nh.advertise <package_delivery::multiDOF_array>("normal_traj", 1);

    ros::Subscriber col_coming_sub = 
        nh.subscribe<package_delivery::BoolPlusHeader>("col_coming", 1, col_coming_callback);
    


    ros::ServiceClient start_profiling_client = 
      nh.serviceClient<profile_manager::start_profiling_srv>("/start_profiling");

    profile_manager::start_profiling_srv start_profiling_srv_inst;
    start_profiling_srv_inst.request.key = "";

    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
    const float goal_s_error_margin = 3.0; //ok distance to be away from the goal.
                                           //this is b/c it's very hard 
                                           //given the issues associated with
                                           //flight controler to land exactly
                                           //on the goal
    
    bool srv_call_status = false;
    trajectory_t normal_traj;
    ros::Time panic_realization_start_t;
    ros::Time panic_realization_end_t;
    msr::airlib::FlightStats init_stats, end_stats;
    std::string mission_status;
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    
    // Wait for the localization method to come online
    waitForLocalization("ground_truth");

    //update_stats_file(stats_file_addr,"\n\n# NEW\n# Package delivery\n###\nTime: ");
    //log_time(stats_file_addr);
    //update_stats_file(stats_file_addr,"###\n");
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    ros::Time loop_start_t(0,0); 
    ros::Time loop_end_t(0,0); //if zero, it's not valid
    geometry_msgs::Twist twist;
    twist.linear.x = twist.linear.y = twist.linear.z = 1;
    geometry_msgs::Twist acceleration;
    acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 1; 
	ros::Rate pub_rate(80);
    for (State state = setup; ros::ok(); ) {
		pub_rate.sleep();
        ros::spinOnce();
        
        State next_state = invalid;
        loop_start_t = ros::Time::now();
        if (state == setup)
        {
            control_drone(drone);

            goal = get_goal();
            start = get_start(drone);
            
            profiling_data_srv_inst.request.key = "start_profiling";
            if (ros::service::waitForService("/record_profiling_data", 10)){ 
                if(!record_profiling_data_client.call(profiling_data_srv_inst)){
                    ROS_ERROR_STREAM("could not probe data using stats manager");
                    ros::shutdown();
                }
            }
            
            spin_around(drone);
            next_state = waiting;
        }
        else if (state == waiting)
        {
            if (CLCT_DATA){ 
                start_hook_t = ros::Time::now(); 
            }
            
            start = get_start(drone);
            normal_traj = request_trajectory(get_trajectory_client, start, goal, 
                    twist, acceleration);
           
            // Profiling
            if (CLCT_DATA){ 
                end_hook_t = ros::Time::now(); 
                /* 
                if (!clcted_col_coming_data){ 
                    g_col_coming_to_req_traj_acc += (start_hook_t - col_coming_time_stamp).toSec()*1e9;
                    clcted_col_coming_data = true; 
                    if (DEBUG){ 
                        ROS_INFO_STREAM("col coming to req traj"<< (start_hook_t - col_coming_time_stamp).toSec());
                    }
                }
                */     
                if (DEBUG){ 
                    ROS_INFO_STREAM("req traj with srv overhead"<<end_hook_t - start_hook_t); 
                }
                g_planning_time_including_ros_overhead_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
                g_planning_ctr++; 
            } 
            
            //std::this_thread::sleep_for(std::chrono::milliseconds(150));
            package_delivery::multiDOF_array array_of_point_msg; 
            
            if (!clcted_col_coming_data){ 
                array_of_point_msg.header.stamp = col_coming_time_stamp;//ros::Time::now();
                //ROS_INFO_STREAM("diff "<<(ros::Time::now() - col_coming_time_stamp).toSec() << " " <<(end_hook_t - start_hook_t)); 
                //array_of_point_msg.header.stamp = ros::Time::now();
                clcted_col_coming_data = true; 
            }else{
                ros::Time temp(0,0); 
                array_of_point_msg.header.stamp = temp;
            }
            
            //ROS_INFO_STREAM("blah blah"<< (ros::Time::now() - col_coming_time_stamp) << " " << ros::Time::now() << " " << col_coming_time_stamp); 
            ros::Time cp_time = ros::Time::now(); 
            for (auto point : normal_traj){
                package_delivery::multiDOF point_msg;
                point_msg.x = point.x;
                point_msg.y = point.y;
                point_msg.z = point.z;
                point_msg.vx = point.vx;
                point_msg.vy = point.vy;
                point_msg.vz = point.vz;
                point_msg.ax = point.ax;
                point_msg.ay = point.ay;
                point_msg.az = point.az;
                point_msg.yaw = point.yaw;
                point_msg.duration = point.duration;
                array_of_point_msg.points.push_back(point_msg); 
            }
            ros::Time cp_end_time = ros::Time::now(); 

            trajectory_pub.publish(array_of_point_msg);

            if (!normal_traj.empty())
                next_state = flying;
            else {
                next_state = trajectory_completed;
            }

            //next_state = failed;
        }
        else if (state == flying)
        {
            // Choose next state (failure, completion, or more flying)
            /* 
               if (slam_lost && created_slam_loss_traj && trajectory_done(slam_loss_traj)) {
               if (reset_slam(drone, "/slam_lost"))
               next_state = trajectory_completed;
               else
               next_state = failed;
               }
               */
            //ros::Time start__t = ros::Time::now(); 
            srv_call_status = follow_trajectory_status_client.call(follow_trajectory_status_srv_inst);
            //ros::Time end__t = ros::Time::now(); 

            //ROS_INFO_STREAM("diff "<<(end__t - start__t)); 
            if(!srv_call_status){
                ROS_INFO_STREAM("could not make a service all to trajectory done");
                next_state = flying;
            }else if (follow_trajectory_status_srv_inst.response.success.data) {
                next_state = trajectory_completed; 
                twist = follow_trajectory_status_srv_inst.response.twist;
                acceleration = follow_trajectory_status_srv_inst.response.acceleration;
            }
            else if (col_coming){
                next_state = trajectory_completed; 
                twist = follow_trajectory_status_srv_inst.response.twist;
                acceleration = follow_trajectory_status_srv_inst.response.acceleration;
                col_coming = false; 
                clcted_col_coming_data = false;
            }
            else{
                next_state = flying;
            }

        }
        else if (state == trajectory_completed)
        {
            fail_ctr = normal_traj.empty() ? fail_ctr+1 : 0; 
            
            if (normal_traj.empty()){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (fail_ctr >fail_threshold) {
                next_state = failed;

            }else if (dist(drone.position(), goal) < goal_s_error_margin) {
                ROS_INFO("Delivered the package and returned!");
                mission_status = "completed"; 
                g_mission_status = mission_status;            
                //update_stats_file(stats_file_addr,"mission_status completed");
                next_state = setup;
                log_data_before_shutting_down();
                signal_supervisor(g_supervisor_mailbox, "kill"); 
                ros::shutdown();
            } else { //If we've drifted too far off from the destination
                //ROS_WARN("We're a little off...");

                auto pos = drone.position();
                // std::cout << "Pos: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
                // std::cout << "Goal: " << goal.x << " " << goal.y << " " << goal.z << std::endl;
                // std::cout << "Dist: " << dist(pos, goal) << std::endl;

                start = get_start(drone);
                next_state = waiting;
            }
        }
        else if (state == failed) {
            ROS_ERROR("Failed to reach destination");
            mission_status = "failed"; 
            g_mission_status = mission_status;            
            log_data_before_shutting_down();
            signal_supervisor(g_supervisor_mailbox, "kill"); 
            ros::shutdown();
            //update_stats_file(stats_file_addr,"mission_status failed");
            next_state = setup;
        }
        else
        {
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
    //collect data before shutting down
    //end_stats = drone.getFlightStats();
    //output_flight_summary(init_stats, end_stats, mission_status, stats_file_addr);
    return 0;
}

