#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <limits>
#include <geometry_msgs/Vector3.h>
#include <mavbench_msgs/multiDOFpoint.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include "Profiling.h"
#include "Drone.h"

void sigIntHandler(int sig);


// Functions and classes to manipulate and follow trajectories
struct multiDOFpoint {
    double x, y, z;
    double vx, vy, vz;
    double ax, ay, az; // Currently, the acceleration values are ignored
    double yaw;
    bool blocking_yaw;
    double duration;
};
typedef std::deque<multiDOFpoint> trajectory_t;
enum yaw_strategy_t { ignore_yaw, face_forward, face_backward, follow_yaw };

trajectory_t create_trajectory_from_msg(const mavbench_msgs::multiDOFtrajectory&);
mavbench_msgs::multiDOFtrajectory create_trajectory_msg(const trajectory_t&);
multiDOFpoint trajectory_at_time(const trajectory_t& traj, double t);
multiDOFpoint trajectory_at_time(const mavbench_msgs::multiDOFtrajectory& traj, double t);
trajectory_t append_trajectory (trajectory_t first, const trajectory_t& second);

double follow_trajectory(Drone& drone, trajectory_t * traj,
        trajectory_t * reverse_traj,
        yaw_strategy_t yaw_strategy = ignore_yaw,
        bool check_position = true,
        float max_speed = std::numeric_limits<double>::infinity(),
        //float max_speed = 3,
        float time = 2); 


// Recovery methods
trajectory_t create_slam_loss_trajectory(Drone& drone, trajectory_t& normal_traj, const trajectory_t& rev_normal_traj);
bool reset_slam(Drone& drone, const std::string& topic);


// Spinning commands
void spin_around(Drone &drone);
void scan_around(Drone &drone, int angle);


// Utility functions
float distance(float x, float y, float z);
float yawFromQuat(geometry_msgs::Quaternion q);
float yawFromVelocity(float vx, float vy);
void waitForLocalization(std::string method);

#endif

