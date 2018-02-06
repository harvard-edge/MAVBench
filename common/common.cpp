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

static bool action_upon_slam_loss_backtrack (Drone& drone, const std::string& topic,
        trajectory_t& traj, trajectory_t& reverse_traj);
static bool action_upon_slam_loss_spin(Drone& drone, const std::string& topic);
static bool action_upon_slam_loss_reset(Drone& drone, const std::string& topic);

static trajectory_t append_trajectory (trajectory_t first, const trajectory_t& second);
static multiDOFpoint reverse_point(multiDOFpoint mdp);
static float yawFromQuat(geometry_msgs::Quaternion q);

template <class T>
static T magnitude(T a, T b, T c) {
    return std::sqrt(a*a + b*b + c*c);
}

template <class T>
static T last_msg (std::string topic) {
    // Return the last message of a latched topic
    return *(ros::topic::waitForMessage<T>(topic));
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
    ros::shutdown();
    //exit(0);
}

trajectory_t create_panic_trajectory(Drone& drone, const geometry_msgs::Vector3& panic_dir)
{
    multiDOFpoint p;

    p.yaw = drone.get_yaw();

    p.vx = panic_dir.x * std::sin(p.yaw*M_PI/180);
    p.vy = panic_dir.y * std::cos(p.yaw*M_PI/180);
    p.vz = panic_dir.z + 0.1; // Counter-act AirSim's slight downward drift

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

    // Add pause to trajectory
    /*
    multiDOFpoint pause_p;
    pause_p.vx = pause_p.vy = pause_p.vz = 0;
    pause_p.duration = 2.0;
    pause_p.yaw = drone.get_yaw();

    result.push_back(pause_p);
    */

    // Add backtrack to trajectory
    double distance_left = 5.0;
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


static bool action_upon_slam_loss_reset(Drone& drone, const std::string& topic) {
    ros::NodeHandle nh;
	ros::ServiceClient reset_client = nh.serviceClient<std_srvs::Trigger>("/slam_reset");
    std_srvs::Trigger srv;

    // Reset the SLAM map
    if (reset_client.call(srv)) {
        ROS_INFO("SLAM resetted succesfully");
    } else {
        ROS_ERROR("Failed to reset SLAM");
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


static bool action_upon_slam_loss_spin(Drone& drone, const std::string& topic) {
    float init_yaw = drone.get_yaw();

    // Spin around until we re-localize
    for (int i = angular_vel; i <= 360; i += angular_vel) {
        // Turn slightly
        int angle = init_yaw + i;

        auto start_turn = std::chrono::system_clock::now();
        drone.set_yaw(angle <= 180 ? angle : angle - 360);

        auto end_turn = start_turn + std::chrono::seconds(1);
        std::this_thread::sleep_until(end_turn);

        // Check whether SLAM is back
        std_msgs::Bool is_lost = last_msg<std_msgs::Bool>(topic);

        if (!is_lost.data)
            return true;
    }

    return false;
}

static bool action_upon_slam_loss_backtrack (Drone& drone, const std::string& topic, trajectory_t& traj, trajectory_t& reverse_traj) {
    const double safe_speed = 0.5;

    while (reverse_traj.size() > 1) {
        follow_trajectory(drone, &reverse_traj, &traj, face_backward, false, safe_speed);

        // Check whether SLAM is back
        std_msgs::Bool is_lost = last_msg<std_msgs::Bool>(topic);
        if (!is_lost.data)
            return true;
    }
    ROS_INFO("done");

    return false;
}

bool action_upon_slam_loss (Drone& drone, slam_recovery_method slm...) {
    va_list args;
    va_start(args, slm);

    const std::string lost_topic = "/slam_lost";
    bool success;

    // Stop the drone
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (slm == spin) {
        success = action_upon_slam_loss_spin(drone, lost_topic);
    } else if (slm == backtrack) {
        trajectory_t& traj = *(va_arg(args, trajectory_t*));
        trajectory_t& reverse_traj = *(va_arg(args, trajectory_t*));
        success = action_upon_slam_loss_backtrack(drone, lost_topic, traj, reverse_traj);
    } else if (slm == reset) {
        success = action_upon_slam_loss_reset(drone, lost_topic);
    }

    va_end(args);

    return success;
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

    float init_yaw = drone.get_yaw();
    for (int i = 0; i <= 360; i += 90) {
        int angle = init_yaw + i;
        drone.set_yaw(angle <= 180 ? angle : angle - 360);
    }
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

        if (check_position) {
            auto pos = drone.position();
            v_x += 0.05*(p.x-pos.x);
            v_y += 0.05*(p.y-pos.y);
            v_z += 0.2*(p.z-pos.z);
        }

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
        double speed = std::sqrt(v_x*v_x + v_y*v_y + v_z*v_z);
        double scale = 1;
        if (speed > max_speed) {
            scale = max_speed / speed;

            v_x *= scale;
            v_y *= scale;
            v_z *= scale;
        }

        // Calculate the time for which these flight commands should run
        double flight_time = p.duration <= time ? p.duration : time;
        double scaled_flight_time = flight_time / scale;

        // Fly for flight_time seconds
        auto segment_start_time = std::chrono::system_clock::now();
        drone.fly_velocity(v_x, v_y, v_z, yaw, scaled_flight_time+0.1); 

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

static float yawFromQuat(geometry_msgs::Quaternion q)
{
	float roll, pitch, yaw;

	// Formulas for roll, pitch, yaw
	// roll = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y) );
	// pitch = asin(2*(q.w*q.y - q.z*q.x));
	yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    yaw = (yaw*180)/3.14159265359;

    return (yaw <= 180 ? yaw : yaw - 360);
}

void output_flight_summary(Drone& drone, const std::string& fname)
{
    auto flight_stats = drone.getFlightStats();

    stringstream stats_ss;

    stats_ss << "{  StateOfCharge: " << flight_stats.state_of_charge << "," << endl;
    stats_ss << "  Voltage: " << flight_stats.voltage << "," << endl;
    stats_ss << "  EnergyConsumed: " << flight_stats.energy_consumed << "," << endl;
    stats_ss << "  DistanceTravelled: " << flight_stats.distance_traveled << "," << endl;
    stats_ss << "  FlightTime: " << flight_stats.flight_time << endl;
    stats_ss << "}" << endl;

    update_stats_file(fname, stats_ss.str());
}

trajectory_t create_trajectory(const trajectory_msgs::MultiDOFJointTrajectory& t)
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

        mdp.yaw = yawFromQuat(it->transforms[0].rotation);

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

