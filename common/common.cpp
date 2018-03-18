#include "common.h"

#include <iostream>
#include <fstream>
#include <string>
#include <deque>
#include <algorithm>
#include <cmath>
#include <cstdarg>

#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "Drone.h"

using namespace std;

static const int angular_vel = 15;

static trajectory_t append_trajectory (trajectory_t first, const trajectory_t& second);
static multiDOFpoint reverse_point(multiDOFpoint mdp);

template <class T>
static T magnitude(T a, T b, T c) {
    return std::sqrt(a*a + b*b + c*c);
}


template <class T>
static T last_msg (std::string topic) {
    // Return the last message of a latched topic
    return *(ros::topic::waitForMessage<T>(topic));
}


void signal_supervisor(std::string file_to_write_to, std::string msg){
    std::ofstream file_to_write_to_h; //file handle write to when completed
    file_to_write_to_h.open(file_to_write_to, std::ofstream::out);
    file_to_write_to_h<< msg;
    file_to_write_to_h.close();
}


void update_stats_file(const std::string& stats_file__addr, const std::string& content){
    std::ofstream myfile;
    myfile.open(stats_file__addr, std::ofstream::out | std::ofstream::app);
    myfile << content << std::endl;
    myfile.close();
    return;
}


void sigIntHandler(int sig)
{
    //ros::shutdown();
    exit(0);
}

trajectory_t create_panic_trajectory(Drone& drone, const geometry_msgs::Vector3& panic_velocity)
{
    multiDOFpoint p;

    p.yaw = drone.get_yaw();

    p.vx = panic_velocity.x * std::cos(M_PI/90 - p.yaw*M_PI/180);
    p.vy = panic_velocity.y * std::cos(p.yaw*M_PI/180);
    p.vz = panic_velocity.z + 0.1; // Counter-act AirSim's slight downward drift

    // p.vx = -std::sin(p.yaw*M_PI/180);
    // p.vy = -std::cos(p.yaw*M_PI/180);
    // p.vz = 0.1; // Counter-act AirSim's slight downward drift

    p.duration = std::numeric_limits<double>::infinity();

    trajectory_t result;
    result.push_back(p);

    return result;
}


trajectory_t create_future_col_trajectory(const trajectory_t& normal_traj, double stopping_distance)
{
    if (normal_traj.empty())
        return trajectory_t();

    trajectory_t result;

    multiDOFpoint first_p = normal_traj.front();

    double initial_velocity = magnitude(first_p.vx, first_p.vy, first_p.vz);
    initial_velocity = std::max(initial_velocity, 1.0);
    double distance_left = stopping_distance;

    for (multiDOFpoint p : normal_traj) {
        double v = magnitude(p.vx, p.vy, p.vz);
        double max_v = initial_velocity * distance_left/stopping_distance;

        if (v*p.duration > distance_left) {
            p.duration = distance_left / v;
        }

        distance_left -= v*p.duration;
        
        double scale = v > max_v ? max_v/v : 1;

        p.vx *= scale;
        p.vy *= scale;
        p.vz *= scale;
        p.duration /= scale;

        result.push_back(p);

        if (distance_left <= 0)
            break;
    }

    return result;
}

trajectory_t create_slam_loss_trajectory(Drone& drone, trajectory_t& normal_traj, const trajectory_t& rev_normal_traj)
{
    trajectory_t result;
    float current_yaw = drone.get_yaw();
    auto current_pos = drone.position();

    // Add pause to trajectory
    multiDOFpoint pause_p;
    pause_p.x = current_pos.x;
    pause_p.y = current_pos.y;
    pause_p.z = current_pos.z;
    pause_p.vx = pause_p.vy = pause_p.vz = 0;
    pause_p.duration = 2.0;
    pause_p.yaw = current_yaw;

    result.push_back(pause_p);

    // Add spinning around to trajectory
    const float scanning_width = 45;

    multiDOFpoint scan_p;
    scan_p.x = current_pos.x;
    scan_p.y = current_pos.y;
    scan_p.z = current_pos.z;
    scan_p.vx = scan_p.vy = scan_p.vz = 0;
    scan_p.duration = scanning_width / drone.maxYawRateDuringFlight();

    float yaws[] = {current_yaw - scanning_width, current_yaw, current_yaw + scanning_width, current_yaw};

    for (float y : yaws) {
        scan_p.yaw = y;
        result.push_back(scan_p);
    }

    // Add backtrack to trajectory
    double distance_left = 500.0; // TODO: make this a reasonable number
    const double safe_v = 1.0;

    for (multiDOFpoint p : rev_normal_traj) {
        double v = magnitude(p.vx, p.vy, p.vz);

        if (v*p.duration > distance_left) {
            p.duration = distance_left / v;
        }

        distance_left -= v*p.duration;
        
        double scale = v > safe_v ? safe_v/v : 1;

        p.vx *= scale;
        p.vy *= scale;
        p.vz *= scale;
        p.duration /= scale;

        result.push_back(p);

        if (distance_left <= 0)
            break;
    }

    // Add one last pause
    multiDOFpoint last_pause_p;
    last_pause_p.x = result.back().x;
    last_pause_p.y = result.back().y;
    last_pause_p.z = result.back().z;
    last_pause_p.vx = last_pause_p.vy = last_pause_p.vz = 0;
    last_pause_p.duration = 2.0;
    last_pause_p.yaw = YAW_UNCHANGED;

    result.push_back(last_pause_p);

    // Slow down normal_traj
    const double max_a = 1.0;
    double max_v = safe_v;

    for (multiDOFpoint& p : normal_traj) {
        double v = magnitude(p.vx, p.vy, p.vz);

        double scale = v > max_v ? max_v/v : 1;

        p.vx *= scale;
        p.vy *= scale;
        p.vz *= scale;
        p.duration /= scale;

        max_v += max_a*p.duration;
    }

    return result;
}

bool reset_slam(Drone& drone, const std::string& topic) {
    ros::NodeHandle nh;
	ros::ServiceClient reset_client = nh.serviceClient<std_srvs::Trigger>("/slam_reset");
    std_srvs::Trigger srv;

    // Reset the SLAM map
    if (reset_client.call(srv)) {
        ROS_INFO("SLAM resetted succesfully");
    } else {
        ROS_ERROR("Failed to reset SLAM");
        return false;
    }

    // Move around a little to initialize SLAM
    drone.fly_velocity(-0.5, 0, 0, 2);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    drone.fly_velocity(0.5, 0, 0, 4);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    drone.fly_velocity(-0.5, 0, 0, 2);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std_msgs::Bool is_lost = last_msg<std_msgs::Bool>(topic);
    return !is_lost.data;
}


float distance(float x, float y, float z) {
  return std::sqrt(x*x + y*y + z*z);
}


void scan_around(Drone &drone, int angle) {
    float init_yaw = drone.get_yaw();
    ROS_INFO("Scanning around from %f degrees...", init_yaw);

    if (angle > 90) {
		ROS_INFO("we don't have support for angles greater than 90");
        exit(0);
	}

    drone.set_yaw(init_yaw+angle <= 180 ? init_yaw + angle : init_yaw + angle - 360);
    drone.set_yaw(init_yaw);
    drone.set_yaw(init_yaw-angle >= -180 ? init_yaw - angle : init_yaw - angle + 360);
    drone.set_yaw(init_yaw);
}


void spin_around(Drone &drone) {
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ROS_INFO("Spinning around...");
    ros::Time last_time;
    float init_yaw = drone.get_yaw();
    double start_z = drone.pose().position.z; // Get drone's current position
    auto start_t = ros::Time::now();
    int angle_corrected;
    for (int i = 0; i <= 360; i += 90) {
        int angle = init_yaw + i;
        angle_corrected  = (angle <= 180 ? angle : angle - 360);
        drone.set_yaw(angle_corrected);
        //drone.set_yaw(angle <= 180 ? angle : angle - 360);
    }

    //to correct 
    double dt = (ros ::Time::now() - start_t).toSec(); 
    double v_z = (start_z - drone.pose().position.z)/dt;
    
    drone.fly_velocity(0, 0, 10*v_z, angle_corrected, dt/10);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}


// Follows trajectory, popping commands off the front of it and returning those commands in reverse order
void follow_trajectory(Drone& drone, trajectory_t * traj,
        trajectory_t * reverse_traj, yaw_strategy_t yaw_strategy,
        bool check_position, float max_speed, float time) {

    trajectory_t reversed_commands;

    
    while (time > 0 && traj->size() > 0) {
        multiDOFpoint p = traj->front();

        // Calculate the velocities we should be flying at
        double v_x = p.vx;
        double v_y = p.vy;
        double v_z = p.vz;
        //ROS_ERROR_STREAM("before before speed scaling"<<v_x<< " "<< v_y << " " <<v_z);
        //ROS_ERROR_STREAM("before correction"<<v_x<< " "<< v_y << " " <<v_z);
         
        if (check_position) {
            auto pos = drone.position();
            v_x += 0.05*(p.x-pos.x);
            v_y += 0.05*(p.y-pos.y);
            v_z += 0.2*(p.z-pos.z);
            if (distance(p.x-pos.y, p.y-pos.y, p.z-pos.z)>2) {
                ROS_ERROR_STREAM("distance greater than 2"); 
            }
            else if (distance(p.x-pos.y, p.y-pos.y, p.z-pos.z)>1) {
                ROS_ERROR_STREAM("distance greater than 1"); 
            }
        }
        
        //ROS_ERROR_STREAM("before scaling"<<v_x<< " "<< v_y << " " <<v_z);
        // Calculate the yaw we should be flying with
        float yaw = p.yaw;
        if (yaw_strategy == ignore_yaw)
            yaw = YAW_UNCHANGED;
        else if (yaw_strategy == face_forward)
            yaw = FACE_FORWARD;
        else if (yaw_strategy == face_backward) {
            yaw = FACE_BACKWARD;
        }

        // Make sure we're not going over the maximum speed
        double speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z)/3);
        double scale = 1;
        //ROS_ERROR_STREAM("BEFORE speed scaling"<<v_x<< " "<< v_y << " " <<v_z);
        if (speed > max_speed) {
            scale = max_speed / speed;
            //ROS_ERROR_STREAM("exceed max speed "<< "max_speed"<<max_speed<< " speed"<<speed<<"scael"<<scale);
            
            v_x *= scale;
            v_y *= scale;
            v_z *= scale;
            ROS_ERROR_STREAM("AFTER speed scaling"<<v_x<< " "<< v_y << " " <<v_z);
            
        }

        // Calculate the time for which these flight commands should run
        double flight_time = p.duration <= time ? p.duration : time;
        double scaled_flight_time = flight_time / scale;

        // Fly for flight_time seconds
        //ROS_ERROR_STREAM("after scaling"<<v_x<< " "<< v_y << " " <<v_z);
        auto segment_start_time = std::chrono::system_clock::now();
        drone.fly_velocity(v_x, v_y, v_z, yaw, scaled_flight_time+0.1); 

        //ROS_ERROR_STREAM("fly with: "<<v_x<< " "<<v_y<<" " <<v_z);

        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>(scaled_flight_time));

        // Push completed command onto reverse-command stack
        multiDOFpoint rev_point = reverse_point(p);
        rev_point.duration = flight_time;
        reversed_commands.push_front(rev_point);

        // Update trajectory
        traj->front().duration -= flight_time;
        if (traj->front().duration <= 0)
            traj->pop_front();

        time -= flight_time;
    }


    if (reverse_traj != nullptr)
        *reverse_traj = append_trajectory(reversed_commands, *reverse_traj);
}


static multiDOFpoint reverse_point(multiDOFpoint mdp) {
    multiDOFpoint result = mdp;

    result.vx = -mdp.vx;
    result.vy = -mdp.vy;
    result.vz = -mdp.vz;

    return result;
}


static trajectory_t append_trajectory (trajectory_t first, const trajectory_t& second) {
    first.insert(first.end(), second.begin(), second.end());
    return first;
}

float yawFromQuat(geometry_msgs::Quaternion q)
{
	float roll, pitch, yaw;

	// Formulas for roll, pitch, yaw
	// roll = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y) );
	// pitch = asin(2*(q.w*q.y - q.z*q.x));
	yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    yaw = (yaw*180)/3.14159265359;

    return (yaw <= 180 ? yaw : yaw - 360);
}

/*
void update_stats(Drone& drone, const std::string& fname, std::string state){
    auto static profiling_data = drone.getFlightStats();

}
*/

trajectory_t create_trajectory(const trajectory_msgs::MultiDOFJointTrajectory& t, bool face_forward)
{
    trajectory_t result;
    for (auto it = t.points.begin(); it+1 != t.points.end(); ++it) {
        multiDOFpoint mdp;

        mdp.x = it->transforms[0].translation.x;
        mdp.y = it->transforms[0].translation.y;
        mdp.z = it->transforms[0].translation.z;

        mdp.vx = it->velocities[0].linear.x;
        mdp.vy = it->velocities[0].linear.y;
        mdp.vz = it->velocities[0].linear.z;

        if (face_forward) {
            if (mdp.vx == 0 && mdp.vy == 0)
                mdp.yaw = YAW_UNCHANGED;
            else
                mdp.yaw = 90 - atan2(mdp.vy, mdp.vx)*180.0/3.14;
        } else {
            mdp.yaw = yawFromQuat(it->transforms[0].rotation);
        }

        mdp.duration = ((it+1)->time_from_start - it->time_from_start).toSec();

        result.push_back(mdp);
    }

    return result;
}

trajectory_msgs::MultiDOFJointTrajectory create_trajectory_msg(const trajectory_t& t)
{
    trajectory_msgs::MultiDOFJointTrajectory result;

    double time_from_start = 0;
    for (const multiDOFpoint& p : t) {
        trajectory_msgs::MultiDOFJointTrajectoryPoint mdp;

		geometry_msgs::Transform pos;
		pos.translation.x = p.x;
		pos.translation.y = p.y;
		pos.translation.z = p.z;

		geometry_msgs::Twist vel;
		vel.linear.x = p.vx;
		vel.linear.y = p.vy;
		vel.linear.z = p.vz;

		ros::Duration dur(time_from_start);

		mdp.transforms.push_back(pos);
		mdp.velocities.push_back(vel);
		mdp.time_from_start = dur;

		result.points.push_back(mdp);

        time_from_start += p.duration;
    }

    // Add final point
    multiDOFpoint last_p = t.back();
    trajectory_msgs::MultiDOFJointTrajectoryPoint mdp;

    geometry_msgs::Transform pos;
    pos.translation.x = last_p.x + last_p.vx*last_p.duration;
    pos.translation.y = last_p.y + last_p.vy*last_p.duration;
    pos.translation.z = last_p.z + last_p.vz*last_p.duration;

    geometry_msgs::Twist vel;
    vel.linear.x = last_p.vx;
    vel.linear.y = last_p.vy;
    vel.linear.z = last_p.vz;

    ros::Duration dur(time_from_start);

    mdp.transforms.push_back(pos);
    mdp.velocities.push_back(vel);
    mdp.time_from_start = dur;

    result.points.push_back(mdp);

    return result;
}

void waitForLocalization(std::string method)
{
    // Wait for the localization method to come online
    tf::TransformListener tfListen;
    while(1) {
        try {
            tf::StampedTransform tf;
            tfListen.lookupTransform("/world", "/"+method, ros::Time(0), tf);
            break;
        } catch(tf::TransformException& ex) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}
