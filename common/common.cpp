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


void sigIntHandler(int sig)
{
    //ros::shutdown();
    exit(0);
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

    int angle_corrected;
    for (int i = 0; i <= 360; i += 90) {
        int angle = init_yaw + i;
        angle_corrected  = (angle <= 180 ? angle : angle - 360);
        drone.set_yaw_at_z(angle_corrected, start_z);
        //drone.set_yaw(angle <= 180 ? angle : angle - 360);
    }

    // to correct 
    double dz = start_z - drone.pose().position.z;
    double vz = dz > 0 ? 1 : -1;
    double dt = dz > 0 ? dz : -dz;
    
    drone.fly_velocity(0, 0, vz, YAW_UNCHANGED, dt);
    std::this_thread::sleep_for(std::chrono::milliseconds(int(dt*1000.0)));
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}


// Follows trajectory, popping commands off the front of it and returning those commands in reverse order
void follow_trajectory(Drone& drone, trajectory_t * traj,
        trajectory_t * reverse_traj, yaw_strategy_t yaw_strategy,
        bool check_position, float max_speed, float time){

    trajectory_t reversed_commands;

    ros::Time start_hook_t;
    while (time > 0 && traj->size() > 0) {
        start_hook_t = ros::Time::now();  
        multiDOFpoint p = traj->front();

        // Calculate the velocities we should be flying at
        double v_x = p.vx;
        double v_y = p.vy;
        double v_z = p.vz;
         
        if (check_position) {
            auto pos = drone.position();
            v_x += 0.2*(p.x-pos.x);
            v_y += 0.2*(p.y-pos.y);
            v_z += 0.5*(p.z-pos.z);
        }
        
        // Calculate the yaw we should be flying with
        float yaw = p.yaw;
        if (yaw_strategy == ignore_yaw)
            yaw = YAW_UNCHANGED;
        else if (yaw_strategy == face_forward)
            yaw = FACE_FORWARD;
        else if (yaw_strategy == face_backward)
            yaw = FACE_BACKWARD;

        // Check whether the yaw needs to be set before we fly
        if (p.blocking_yaw)
            drone.set_yaw(p.yaw);

        // Make sure we're not going over the maximum speed
        double speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z));
        double scale = 1;
        if (speed > max_speed) {
            scale = max_speed / speed;
            
            v_x *= scale;
            v_y *= scale;
            v_z *= scale;
            speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z));
        }

        // Calculate the time for which these flight commands should run
        double flight_time = p.duration <= time ? p.duration : time;
        double scaled_flight_time = flight_time / scale;

        // Fly for flight_time seconds
        auto segment_start_time = std::chrono::system_clock::now();
        drone.fly_velocity(v_x, v_y, v_z, yaw, scaled_flight_time); 
        
        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>(scaled_flight_time));

        // Push completed command onto reverse-command stack
        multiDOFpoint rev_point = reverse_point(p);
        rev_point.duration = flight_time;
        reversed_commands.push_front(rev_point);

        // Update trajectory
        traj->front().duration -= flight_time;
        if (traj->front().duration <= 0){
            traj->pop_front();
        }

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


trajectory_t append_trajectory (trajectory_t first, const trajectory_t& second) {
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

trajectory_t create_trajectory_from_msg(const mavbench_msgs::multiDOFtrajectory& t)
{
    trajectory_t result;
    for (const auto& mdp_msg : t.points) {
        multiDOFpoint mdp;

        mdp.x = mdp_msg.x;
        mdp.y = mdp_msg.y;
        mdp.z = mdp_msg.z;

        mdp.vx = mdp_msg.vx;
        mdp.vy = mdp_msg.vy;
        mdp.vz = mdp_msg.vz;

        mdp.ax = mdp_msg.ax;
        mdp.ay = mdp_msg.ay;
        mdp.az = mdp_msg.az;

        mdp.yaw = mdp_msg.yaw;
        mdp.blocking_yaw = mdp_msg.blocking_yaw;

        mdp.duration = mdp_msg.duration;

        result.push_back(mdp);
    }

    return result;
}

mavbench_msgs::multiDOFtrajectory create_trajectory_msg(const trajectory_t& t)
{
    mavbench_msgs::multiDOFtrajectory result;

    result.append = false;
    result.reverse = false;

    for (const auto& mdp : t) {
        mavbench_msgs::multiDOFpoint mdp_msg;

        mdp_msg.x = mdp.x;
        mdp_msg.y = mdp.y;
        mdp_msg.z = mdp.z;

        mdp_msg.vx = mdp.vx;
        mdp_msg.vy = mdp.vy;
        mdp_msg.vz = mdp.vz;

        mdp_msg.ax = mdp.ax;
        mdp_msg.ay = mdp.ay;
        mdp_msg.az = mdp.az;

        mdp_msg.yaw = mdp.yaw;
        mdp_msg.blocking_yaw = mdp.blocking_yaw;

        mdp_msg.duration = mdp_msg.duration;

        result.points.push_back(mdp_msg);
    }

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

float yawFromVelocity(float vx, float vy)
{
    if (vx == 0 && vy == 0)
        return YAW_UNCHANGED;
    return 90 - atan2(vy, vx)*180.0/3.14;
}

multiDOFpoint trajectory_at_time(const trajectory_t& traj, double t)
{
    for (const auto& mdofp : traj) {
        t -= mdofp.duration;

        if (t <= 0)
            return mdofp;
    }

    return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0};
}

multiDOFpoint trajectory_at_time(const mavbench_msgs::multiDOFtrajectory& traj, double t)
{
    for (const auto& mdofp : traj.points) {
        t -= mdofp.duration;

        if (t <= 0)
            return {mdofp.x, mdofp.y, mdofp.z, mdofp.vx, mdofp.vy, mdofp.vz,
                mdofp.ax, mdofp.ay, mdofp.az, mdofp.yaw, mdofp.blocking_yaw,
                mdofp.duration};
    }

    return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0};
}

