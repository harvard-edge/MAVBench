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

#include <visualization_msgs/Marker.h>
#include <thread>
#include <chrono>
#include "coord.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <multiagent_collision_check/Segment.h>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nbvplanner/nbvp_srv.h>
#include "common/Common.hpp"
#include <fstream>
#include "Drone.h"
#include "control_drone.h"
#include "common.h"

visualization_msgs::Marker path_to_follow_marker;
std::string g_stats_file_addr;
bool g_slam_lost = false;
//data to be logged in stats manager
std::string g_mission_status = "failed";
float g_coverage = 0 ;
float g_path_computation_time = 0;
float g_path_computation_time_avg = 0;
float g_path_computation_time_acc = 0;
int g_iteration = 0;
long long g_accumulate_loop_time_ms = 0; //it is in ms
int g_loop_ctr = 0; 
bool g_start_profiling = false; 
std::string g_supervisor_mailbox; //file to write to when completed
float g_max_yaw_rate;
float g_max_yaw_rate_during_flight;



void log_data_before_shutting_down(){
    profile_manager::profiling_data_srv profiling_data_srv_inst;

    std::string ns = ros::this_node::getName();
    profiling_data_srv_inst.request.key = ns+"_mean_loop_time";
    profiling_data_srv_inst.request.value = ((g_accumulate_loop_time_ms)/1000)/g_loop_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "mission_status";
    profiling_data_srv_inst.request.value = (g_mission_status == "completed" ? 1.0: 0.0);
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "coverage";
    profiling_data_srv_inst.request.value = g_coverage;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "g_path_computation_time_avg";
    profiling_data_srv_inst.request.value = g_path_computation_time_acc/g_iteration;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "g_path_computation_time_acc";
    profiling_data_srv_inst.request.value = g_path_computation_time_acc;
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
        signal_supervisor(g_supervisor_mailbox, "kill"); 
        ros::shutdown();
    }
    exit(0);
}


void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    g_slam_lost = msg->data;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mapping");
  ros::NodeHandle nh;
  signal(SIGINT, sigIntHandlerPrivate);
  ros::Publisher trajectory_pub = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory
      > (mav_msgs::default_topics::COMMAND_TRAJECTORY, 5);
  
  ros::ServiceClient record_profiling_data_client = 
      nh.serviceClient<profile_manager::profiling_data_srv>("/record_profiling_data");
  
  ros::ServiceClient start_profiling_client = 
      nh.serviceClient<profile_manager::start_profiling_srv>("/start_profiling");
  
   ros::Subscriber slam_lost_sub = 
		nh.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
    
  profile_manager::start_profiling_srv start_profiling_srv_inst;
  start_profiling_srv_inst.request.key = "";
  bool clct_data = true;
  uint16_t port = 41451;
  std::string ip_addr__global;
  std::string localization_method; 
  std::string ns = ros::this_node::getName();
  if (!ros::param::get("/ip_addr", ip_addr__global)) {
    ROS_FATAL("Could not start mapping. Parameter missing! Looking for %s",
              (ns + "/ip_addr").c_str());
    return -1;
  }
    if(!ros::param::get("/localization_method",localization_method))  {
      ROS_FATAL_STREAM("Could not start mapping localization_method not provided");
      return -1;
    }

    if(!ros::param::get("/stats_file_addr",g_stats_file_addr)){
        ROS_FATAL("Could not start mapping . Parameter missing! Looking for %s", 
                (ns + "/g_stats_file_addr").c_str());
    }

  float coverage_threshold;
  if (!ros::param::get("/coverage_threshold", coverage_threshold)) {
    ROS_FATAL("Could not start mapping. Parameter missing! Looking for %s",
              (ns + "/coverage_threshold").c_str());
    return -1;
  }

  if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
      ROS_FATAL_STREAM("Could not start mapping supervisor_mailbox not provided");
      return -1;
  }

  if(!ros::param::get("/max_yaw_rate",g_max_yaw_rate))  {
      ROS_FATAL_STREAM("Could not start mapping, max_yaw_rate not provided");
      return -1;
  }

  if(!ros::param::get("/max_yaw_rate_during_flight",g_max_yaw_rate_during_flight))  {
      ROS_FATAL_STREAM("Could not start mapping,  max_yaw_rate_during_flight not provided");
      return -1;
  }

  //behzad change for visualization purposes
  ros::Publisher path_to_follow_marker_pub = nh.advertise<visualization_msgs::Marker>("path_to_follow_topic", 1000);
  geometry_msgs::Point p_marker;
  path_to_follow_marker.header.frame_id = "world";
  path_to_follow_marker.type = visualization_msgs::Marker::CUBE_LIST;
  path_to_follow_marker.action = visualization_msgs::Marker::ADD;
  path_to_follow_marker.scale.x = 0.3;


  //ROS_INFO_STREAM("ip address is"<<ip_addr__global); 
  //ROS_ERROR_STREAM("blah"<<ip_addr__global);
  Drone drone(ip_addr__global.c_str(), port, localization_method,
              g_max_yaw_rate, g_max_yaw_rate_during_flight);

  
  //dummy segment publisher
  ros::Publisher seg_pub = nh.advertise <multiagent_collision_check::Segment>("evasionSegment", 1);

  std_srvs::Empty srv;
  //bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  /* 
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }
  */
  
  double dt; //= 1.0;
  double yaw_t; 
  //std::string ns = ros::this_node::getName();
  if (!ros::param::get(ns + "/nbvp/dt", dt)) {
    ROS_FATAL("Could not start mapping. Parameter missing! Looking for %s",
              (ns + "/nbvp/dt").c_str());
    return -1;
  }
  
  //behzad change using segment_dedicated_time instead of dt
  //ros::param::get("/follow_trajectory/yaw_t",yaw_t);
  if (!ros::param::get(ns + "/follow_trajectory/yaw_t",yaw_t)){
      ROS_FATAL_STREAM("Could not start mapping. Parameter missing! Looking for"<<
              "/follow_trajectory/yaw_t");
      return -1;
  }
  double t_offset; 
  if (!ros::param::get(ns + "/nbvp/t_offset",t_offset)){
      ROS_FATAL_STREAM("Could not start mapping. Parameter missing! Looking for"<<
              "/nbvp/t_offset");
      return -1;
  }

  // Wait for the localization method to come online
  waitForLocalization(localization_method);

  double segment_dedicated_time = yaw_t + dt;
  control_drone(drone);

  static int n_seq = 0;

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

  // Wait for 5 seconds to let the Gazebo GUI show up.
  //ros::Duration(5.0).sleep();

  // This is the initialization motion, necessary that the known free space allows the planning
  // of initial paths.
  //ROS_INFO("Starting the planner: Performing initialization motion");

 /* 
  for (double i = 0; i <= 1.0; i = i + 0.25) {
    
      
    //nh.param<double>("wp_x", trajectory_point.position_W.x(), 0.0);
    //nh.param<double>("wp_y", trajectory_point.position_W.y(), 0.0);
    //nh.param<double>("wp_z", trajectory_point.position_W.z(), 1.0);
   
    //behzad change, so the starting point is adjusted based on the drone 
    // starting postion (as opposed to being hardcoded)
    trajectory_point.position_W.x() = drone.pose().position.x;
    trajectory_point.position_W.y() = drone.pose().position.y;
    trajectory_point.position_W.z() = drone.pose().position.z;
    //ROS_INFO_STREAM("blah "<< trajectory_point.position_W.x()  <<" " << trajectory_point.position_W.y()  <<" " << trajectory_point.position_W.z());

    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), -2*M_PI * i);
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(1.0).sleep();
  }
  */
  profile_manager::profiling_data_srv profiling_data_srv_inst;
  profiling_data_srv_inst.request.key = "start_profiling";
  if (ros::service::waitForService("/record_profiling_data", 10)){ 
     if(!record_profiling_data_client.call(profiling_data_srv_inst)){
         ROS_ERROR_STREAM("could not probe data using stats manager");
         ros::shutdown();
     }
  }
  spin_around(drone);
  // Move back a little bit
  int ctr =0; 
  while(ctr < 10) { 
      auto cur_pos = drone.position();
      trajectory_point.position_W.x() = cur_pos.x - .2;
      trajectory_point.position_W.y() = cur_pos.y - .2;
      trajectory_point.position_W.z() = cur_pos.z;
      samples_array.header.seq = n_seq;
      samples_array.header.stamp = ros::Time::now();
      samples_array.points.clear();
      n_seq++;
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
      samples_array.points.push_back(trajectory_point_msg);
      trajectory_pub.publish(samples_array);
      ros::Duration(1).sleep();
      ctr++;
  }


  // Start planning: The planner is called and the computed path sent to the controller.
  g_iteration = 0;
  multiagent_collision_check::Segment dummy_seg;
  ros::ServiceClient nbvplanner_client= 
        nh.serviceClient<nbvplanner::nbvp_srv>("nbvplanner", true);
  
  ros::Time loop_start_t(0,0); 
  ros::Time loop_end_t(0,0); //if zero, it's not valid
  mav_msgs::EigenTrajectoryPoint last_trajectory_point;
  last_trajectory_point.position_W.x() = drone.pose().position.x;
  last_trajectory_point.position_W.y() = drone.pose().position.y;
  last_trajectory_point.position_W.z() = drone.pose().position.z;
  
  int time_out_ctr_threshold = 20; 
  const float goal_s_error_margin = 3.0; //ok distance to be away from the goal.
  int time_out_ctr = 0;

  while (ros::ok()) {
    loop_start_t = ros::Time::now();
    
    if (g_slam_lost) { //skip the iteration
        continue;
    }
    
    ROS_INFO_THROTTLE(0.5, "Planning iteration %i", g_iteration);
    nbvplanner::nbvp_srv planSrv;
    planSrv.request.header.stamp = ros::Time::now();
    planSrv.request.header.seq = g_iteration;
    planSrv.request.header.frame_id = "world";
  
    while ( (distance(drone.pose().position.x - last_trajectory_point.position_W.x(),
                drone.pose().position.y - last_trajectory_point.position_W.y(),
                0) > goal_s_error_margin) && time_out_ctr < time_out_ctr_threshold ){
        /* 
        ROS_INFO_STREAM("drone pos"<<drone.pose().position.x << " " << last_trajectory_point.position_W.x()); 
        ROS_INFO_STREAM("drone pos"<<drone.pose().position.y << " " << last_trajectory_point.position_W.y()); 
        ROS_INFO_STREAM("drone pos"<<drone.pose().position.z << " " << last_trajectory_point.position_W.z()); 
        */ 
        time_out_ctr +=1; 
        ros::Duration(.3).sleep();
    }
    time_out_ctr = 0;

    if(nbvplanner_client.call(planSrv)){ 
        n_seq++;
        
        /*
        ROS_INFO_STREAM("first timne"); 
        ROS_INFO_STREAM("drone pos"<<drone.pose().position.x << " " << last_trajectory_point.position_W.x()); 
        ROS_INFO_STREAM("drone pos"<<drone.pose().position.y << " " << last_trajectory_point.position_W.y()); 
        ROS_INFO_STREAM("drone pos"<<drone.pose().position.z << " " << last_trajectory_point.position_W.z()); 
        */
        //while last time path is not all the way fulfileld, spin
         
                if (planSrv.response.path.size() == 0) {
            ROS_ERROR("path size is zero");
            ros::Duration(1.0).sleep();
        }
        for (int i = 0; i < planSrv.response.path.size(); i++) {
            
            //remember last point, for comparison
            if(i ==  planSrv.response.path.size() - 1) { 
                last_trajectory_point.position_W.x() = planSrv.response.path[i].position.x;
                last_trajectory_point.position_W.y() = planSrv.response.path[i].position.y;
                last_trajectory_point.position_W.y() = planSrv.response.path[i].position.y;
            }

            samples_array.header.seq = n_seq;
            samples_array.header.stamp = ros::Time::now();
            samples_array.header.frame_id = "world";
            samples_array.points.clear();
            tf::Pose pose;
            tf::poseMsgToTF(planSrv.response.path[i], pose);
            double yaw = tf::getYaw(pose.getRotation());
            trajectory_point.position_W.x() = planSrv.response.path[i].position.x;
            trajectory_point.position_W.y() = planSrv.response.path[i].position.y;
            // Add offset to account for constant tracking error of controller
            trajectory_point.position_W.z() = planSrv.response.path[i].position.z + 0.25;
            tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
            trajectory_point.setFromYaw(tf::getYaw(quat));
            mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);

            //behzad change for visualization purposes 
            p_marker.x = planSrv.response.path[i].position.x;
            p_marker.y = planSrv.response.path[i].position.y;
            p_marker.z = planSrv.response.path[i].position.z;
            path_to_follow_marker.points.push_back(p_marker);
            //ROS_INFO_STREAM("TRAJECTORY PTS:"<< i<< " " << p_marker.x << " " << p_marker.y  << " " << p_marker.z);

            std_msgs::ColorRGBA c;
            c.g = 0; c.r = 1; c.b = 1;c.a = 1;
            path_to_follow_marker.colors.push_back(c);
            path_to_follow_marker_pub.publish(path_to_follow_marker);

            samples_array.points.push_back(trajectory_point_msg);
            trajectory_pub.publish(samples_array);
            // ros::Duration(1).sleep();
            // ros::Duration(t_offset + segment_dedicated_time).sleep(); //changed, make sure segmentation time is smaller
        }
    } else {
        ROS_WARN_THROTTLE(1, "Planner not reachable");
        ros::Duration(t_offset + segment_dedicated_time).sleep(); //changed, make sure segmentation time is smaller
        //than 1.5*dt, this way we can finish up the command 
        //before sending out another one
    }
    g_iteration++;
    g_coverage =  planSrv.response.coverage;
    g_path_computation_time = planSrv.response.path_computation_time; 
    g_path_computation_time_acc += g_path_computation_time;    
    if(g_coverage > coverage_threshold){
        g_mission_status = "completed";
        log_data_before_shutting_down();
        signal_supervisor(g_supervisor_mailbox, "kill"); 
        ros::shutdown(); 
    }

    loop_end_t = ros::Time::now(); 
    if (clct_data) { 
        if (ros::service::waitForService("/start_profiling", 10)){ 
            if(!start_profiling_client.call(start_profiling_srv_inst)){
                ROS_ERROR_STREAM("could not probe data using stats manager");
                ros::shutdown();
            }
            //ROS_INFO_STREAM("now it is true");
            g_start_profiling = start_profiling_srv_inst.response.start; 
        }
    } 
    if (g_start_profiling){ 
        if (loop_end_t.isValid()) {
            g_accumulate_loop_time_ms += ((loop_end_t - loop_start_t).toSec())*1000;
            g_loop_ctr++; 
        }
    }
  }
}
