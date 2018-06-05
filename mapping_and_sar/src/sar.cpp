/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <thread>
#include <chrono>
#include "coord.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <multiagent_collision_check/Segment.h>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <mav_msgs/conversions.h>
#include <nbvplanner/nbvp_srv.h>
#include "control_drone.h"
#include "common.h"
#include <mavbench_msgs/future_collision.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/multiDOFpoint.h>
#include <mapping_and_sar/OD.h>

enum State { planning, waiting, completed, invalid };

// Global variables
mavbench_msgs::multiDOFtrajectory g_next_steps_msg;
int g_future_col_seq = 0;
int g_trajectory_seq = 0;
int g_trajectory_future_col_seq = 0;

// Parameters
bool clct_data = true;
std::string ip_addr__global;
std::string localization_method; 
double distance_from_goal_threshold = 0;
float coverage_threshold;
float g_v_max, g_a_max;
float g_max_yaw_rate, g_max_yaw_rate_during_flight;
float g_sensor_max_range;
double g_dt;
// bool g_slam_lost = false;

// Profiling variables
std::string g_supervisor_mailbox; //file to write to when completed
std::string g_stats_file_addr;
int g_reached_time_out = 0;
long g_time_out_ctr_acc = 0;
std::string g_mission_status = "failed";
float g_coverage = 0 ;
float g_path_computation_time = 0;
float g_path_computation_time_avg = 0;
float g_path_computation_time_acc = 0;
int g_iteration = 0;
long long g_accumulate_loop_time = 0; //it is in ms
int g_loop_ctr = 0; 
bool g_start_profiling = false; 
long long g_motion_planning_plus_srv_call_acc = 0;


void log_data_before_shutting_down() {
    int mission_status = 0;
    if (g_mission_status == "completed")
        mission_status = 1;
    else if (g_mission_status == "failed_to_start")
        mission_status = 4;

    if (!log_data_in_profiler("mission_status", mission_status)) {
        ROS_ERROR_STREAM("could not probe data using stats manager");
        ros::shutdown();
    }

    if (!log_data_in_profiler("mapping_main_loop", (((double)g_accumulate_loop_time)/1e9)/g_loop_ctr)) {
        ROS_ERROR_STREAM("could not probe data using stats manager");
        ros::shutdown();
    }

    if (!log_data_in_profiler("motion_planning_plus_srv_call", (((double)g_motion_planning_plus_srv_call_acc)/1e9)/g_iteration)) {
        ROS_ERROR_STREAM("could not probe data using stats manager");
        ros::shutdown();
    }

    if (!log_data_in_profiler("reached_time_out_ctr", g_reached_time_out)) {
        ROS_ERROR_STREAM("could not probe data using stats manager");
        ros::shutdown();
    }

    if (!log_data_in_profiler("time_out_ctr_avg", g_time_out_ctr_acc/g_iteration)) {
        ROS_ERROR_STREAM("could not probe data using stats manager");
        ros::shutdown();
    }


    if (!log_data_in_profiler("coverage", g_coverage)) {
        ROS_ERROR_STREAM("could not probe data using stats manager");
        ros::shutdown();
    }

    if (!log_data_in_profiler("motion_planning_kernel", g_path_computation_time_acc/g_iteration)) {
        ROS_ERROR_STREAM("could not probe data using stats manager");
        ros::shutdown();
    }

    if (!log_data_in_profiler("motion_planning_kernel_acc", g_path_computation_time_acc)) {
        ROS_ERROR_STREAM("could not probe data using stats manager");
        ros::shutdown();
    }

}


void shutdown_app () {
    log_data_before_shutting_down(); 
    signal_supervisor(g_supervisor_mailbox, "kill"); 
    ros::shutdown();
}


void sigIntHandlerPrivate(int signo) {
    if (signo == SIGINT) {
        shutdown_app();
    }
    exit(0);
}


/*
void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    g_slam_lost = msg->data;
}
*/


void future_col_callback (const mavbench_msgs::future_collision::ConstPtr& msg) {
    if (msg->future_collision_seq > g_future_col_seq)
        g_future_col_seq = msg->future_collision_seq;
}


void OD_callback(const mapping_and_sar::OD::ConstPtr& msg){
    if(msg->found) {
       g_mission_status = "completed";
       shutdown_app();
    }
}


void next_steps_callback(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg)
{
    g_next_steps_msg = *msg;
}


void initialize_params() {
    std::string ns = ros::this_node::getName();

    if (!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL("Could not start mapping. Parameter missing! Looking for %s",
              (ns + "/ip_addr").c_str());
        shutdown_app();
        exit(-1);
    }

    if (!ros::param::get("/sensor_max_range", g_sensor_max_range)) {
        ROS_FATAL("Could not start mapping. Parameter missing! Looking for %s",
              (ns + "/sensor_max_range").c_str());
        shutdown_app();
        exit(-1);
    }

    if (!ros::param::get("/distance_from_goal_threshold", distance_from_goal_threshold)) {
        ROS_FATAL("Could not start mapping. Parameter missing! Looking for %s",
              (ns + "/distance_from_goal_threshold").c_str());
        shutdown_app();
        exit(-1);
    }

    if(!ros::param::get("/localization_method",localization_method))  {
        ROS_FATAL_STREAM("Could not start mapping localization_method not provided");
        shutdown_app();
        exit(-1);
    }

    if(!ros::param::get("/stats_file_addr",g_stats_file_addr)){
        ROS_FATAL("Could not start mapping . Parameter missing! Looking for %s", 
            (ns + "/g_stats_file_addr").c_str());
        shutdown_app();
        exit(-1);
    }

    if (!ros::param::get("/coverage_threshold", coverage_threshold)) {
        ROS_FATAL("Could not start mapping. Parameter missing! Looking for %s",
              (ns + "/coverage_threshold").c_str());
        shutdown_app();
        exit(-1);
    }

    if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
        ROS_FATAL_STREAM("Could not start mapping supervisor_mailbox not provided");
        shutdown_app();
        exit(-1);
    }

    if(!ros::param::get("/v_max", g_v_max))  {
        ROS_FATAL_STREAM("Could not start mapping, vmax not provided");
        shutdown_app();
        exit(-1);
    }

    if(!ros::param::get("/a_max", g_a_max))  {
        ROS_FATAL_STREAM("Could not start mapping, amax not provided");
        shutdown_app();
        exit(-1);
    }

    if(!ros::param::get("/max_yaw_rate",g_max_yaw_rate))  {
        ROS_FATAL_STREAM("Could not start mapping, max_yaw_rate not provided");
        shutdown_app();
        exit(-1);
    }

    if(!ros::param::get("/max_yaw_rate_during_flight",g_max_yaw_rate_during_flight))  {
        ROS_FATAL_STREAM("Could not start mapping,  max_yaw_rate_during_flight not provided");
        shutdown_app();
        exit(-1);
    }

    if (!ros::param::get(ns + "/nbvp/dt", g_dt)) {
        ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
                (ns + "/nbvp/dt").c_str());
        shutdown_app();
        exit(-1);
    }
}


bool current_trajectory_collides () {
    return g_trajectory_future_col_seq < g_future_col_seq;
}


bool drone_stopped()
{
    return g_next_steps_msg.trajectory_seq >= g_trajectory_seq &&
        g_next_steps_msg.points.size() == 0;
}


void convert_pose_vector_to_trajectory_msg(const std::vector<geometry_msgs::Pose>& poses, mavbench_msgs::multiDOFtrajectory& result)
{
    for (int i = 1; i < poses.size(); i++) {
        const auto& pose = poses[i];

        // Set position
        mavbench_msgs::multiDOFpoint point;
        point.x = pose.position.x;
        point.y = pose.position.y;
        point.z = pose.position.z;

        // Set velocity
        // if (i != 0) {
        const auto& prev_pose  = poses[i-1];
        point.vx = (pose.position.x - prev_pose.position.x) / g_dt;
        point.vy = (pose.position.y - prev_pose.position.y) / g_dt;
        point.vz = (pose.position.z - prev_pose.position.z) / g_dt;
        // }

        // Set yaw
        tf::Pose tf_pose;
        tf::poseMsgToTF(pose, tf_pose);
        double yaw = tf::getYaw(tf_pose.getRotation());
        tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
        point.yaw = tf::getYaw(quat);
        point.blocking_yaw = false;

        // Set duration
        point.duration = g_dt;

        result.points.push_back(point);
    }
}


void decelerate_end_of_trajectory(mavbench_msgs::multiDOFtrajectory& trajectory) {
    double speed = 0;
    for (auto it = trajectory.points.rbegin(); it != trajectory.points.rend(); ++it) {
        speed += g_a_max * (it->duration);
        if (speed > g_v_max)
            break;

        double v = std::sqrt((it->vx)*(it->vx)+(it->vy)*(it->vy)+(it->vz)*(it->vz));
        if (v > speed) {
            double scale = v / speed;

            it->vx /= scale;
            it->vy /= scale;
            it->vz /= scale;

            it->duration *= scale;
        }
    }
}


void visualize_trajectory(const mavbench_msgs::multiDOFtrajectory& trajectory, ros::Publisher& pub) {
    visualization_msgs::Marker path_to_follow_marker;

    for (const auto& p : trajectory.points) {
        geometry_msgs::Point p_marker;

        p_marker.x = p.x;
        p_marker.y = p.y;
        p_marker.z = p.z;

        std_msgs::ColorRGBA c;
        c.g = 0; c.r = 1; c.b = 1; c.a = 1;
        path_to_follow_marker.colors.push_back(c);

        path_to_follow_marker.points.push_back(p_marker);

        pub.publish(path_to_follow_marker);
    }
}


mavbench_msgs::multiDOFtrajectory nbvp_trajectory(Drone& drone, ros::ServiceClient& nbvp_client) {
    // Prepare a service request to the mapping planner
    nbvplanner::nbvp_srv planSrv;
    planSrv.request.header.stamp = ros::Time::now();
    planSrv.request.header.frame_id = "world";
    planSrv.request.header.seq = g_iteration;
    g_iteration++;

    // If there is a collision detected, then plan from the drone's current position
    planSrv.request.exact_root = !current_trajectory_collides();

    // Make the actual service request to the mapping planner
    ros::Time start_hook_t = ros::Time::now();
    if (!nbvp_client.call(planSrv)) {
        ROS_ERROR("Planner not reachable");
        shutdown_app();
    }
    ros::Time end_hook_t = ros::Time::now();

    // Make sure we're not stuck at the beginning of the map, unable to
    // make any plans
    static int empty_path_ctr = 0;
    static bool failed_to_plan_even_once = true;

    if (planSrv.response.path.size() == 0) {
        ROS_WARN("Path size is 0");
        empty_path_ctr++;

        if (empty_path_ctr > 10 && failed_to_plan_even_once) {
            g_mission_status = "failed_to_start";
            shutdown_app();
        }
    }

    // Record the coverage of the map so far
    g_coverage = planSrv.response.coverage;

    // Convert from the nbvp's message format to MAVBench's message format
    mavbench_msgs::multiDOFtrajectory result;
    convert_pose_vector_to_trajectory_msg(planSrv.response.path, result);

    result.append = true;
    result.reverse = 0;
    result.future_collision_seq = g_trajectory_future_col_seq = g_future_col_seq;
    result.trajectory_seq = g_trajectory_seq;
    g_trajectory_seq++;

    // Make the trajectory decelerate trapezoidally at the end, to prevent sudden, unstable stops
    decelerate_end_of_trajectory(result);

    // Profiling
    g_motion_planning_plus_srv_call_acc += (end_hook_t -start_hook_t).toSec()*1e9;
    g_path_computation_time = planSrv.response.path_computation_time; 
    g_path_computation_time_acc += g_path_computation_time;    

    return result;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);

    initialize_params();

    // Variables
    mavbench_msgs::multiDOFpoint goal;
    const int time_out_ctr_threshold = 10; 
    int time_out_ctr = 0;

    // Profiling variables
    ros::Time start_hook_t, end_hook_t;
    ros::Time loop_start_t(0,0); 
    ros::Time loop_end_t(0,0); //if zero, it's not valid

    // Topics and services
    ros::Publisher trajectory_pub = nh.advertise<mavbench_msgs::multiDOFtrajectory>("/multidoftraj", 5);
    ros::Publisher path_to_follow_marker_pub = nh.advertise<visualization_msgs::Marker>("path_to_follow_topic", 1000);

    ros::ServiceClient nbvplanner_client = 
        nh.serviceClient<nbvplanner::nbvp_srv>("nbvplanner");
    ros::ServiceClient record_profiling_data_client = 
        nh.serviceClient<profile_manager::profiling_data_srv>("/record_profiling_data");
    ros::ServiceClient start_profiling_client = 
        nh.serviceClient<profile_manager::start_profiling_srv>("/start_profiling");

    ros::Subscriber obj_det_sub =
        nh.subscribe("/OD_topic", 2, OD_callback);
    ros::Subscriber future_col_sub =
        nh.subscribe("/col_coming", 1, future_col_callback);
    ros::Subscriber next_steps_sub =
        nh.subscribe("next_steps", 1, next_steps_callback);
    // ros::Subscriber slam_lost_sub =
    //    nh.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);

    // Initialize Drone object
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight);

    // Initialize the drone
    waitForLocalization("ground_truth");
    bool initialized_correctly = control_drone(drone);

    if (!initialized_correctly) {
        g_mission_status = "failed_to_start";
        shutdown_app();
    }

    // Start the profiler
    if (!log_data_in_profiler("start_profiling", 0)) {
        ROS_ERROR_STREAM("could not probe data using stats manager");
        ros::shutdown();
    }

    // Start ros::ok() loop
    for (State state = planning, next_state = invalid;
          ros::ok(); state = next_state)
    {
        loop_start_t = ros::Time::now();
        ros::spinOnce();

        if (state == planning)
        {
            mavbench_msgs::multiDOFtrajectory trajectory = nbvp_trajectory(drone, nbvplanner_client);
            
            trajectory_pub.publish(trajectory);
            visualize_trajectory(trajectory, path_to_follow_marker_pub);

            if (trajectory.points.size() > 0) {
                goal = trajectory.points.back();
                next_state = waiting;
            } else
                next_state = planning;
        }
        else if (state == waiting)
        {
            // Check if a collision has been detected, or if the drone has
            // already completed its path
            if (current_trajectory_collides() || drone_stopped())
                next_state = planning;
            // Check whether we're close enough to the end to replan our next
            // leg of the trajectory
            else {
               auto current_pos = drone.position(); 
               double distance_from_goal = distance(current_pos.x - goal.x,
                       current_pos.y - goal.y, current_pos.z - goal.z);

               if (distance_from_goal <= (1-distance_from_goal_threshold) * g_sensor_max_range) {
                   next_state = planning;
               } else {
                   ros::Duration(0.1).sleep();
                   next_state = waiting;
               }
            }
        }
        else if (state == completed)
        {
            std::cout << "\n\nCoverage: " << g_coverage << "\n\n";
            g_mission_status = "completed";
            shutdown_app();
        }
        else
        {
            ROS_ERROR("Invalid state reached!");
            shutdown_app();
        }

        // Profiling
        loop_end_t = ros::Time::now(); 
        if (clct_data) {
            if(!g_start_profiling) { 
                if (ros::service::waitForService("/start_profiling", 10)){ 
                    profile_manager::start_profiling_srv start_profiling_srv_inst;
                    start_profiling_srv_inst.request.key = "";

                    if(!start_profiling_client.call(start_profiling_srv_inst)){
                        ROS_ERROR_STREAM("could not probe data using stats manager");
                        ros::shutdown();
                    }
                    //ROS_INFO_STREAM("now it is true");
                    g_start_profiling = start_profiling_srv_inst.response.start; 
                }
            } else {
                if (loop_end_t.isValid()) {
                    g_accumulate_loop_time += (((loop_end_t - loop_start_t).toSec())*1e9);
                    g_loop_ctr++; 
                }
            }
        }
    }
}

