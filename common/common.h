#ifndef COMMON_H
#define COMMON_H

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include "Drone.h"

void sigIntHandler(int sig);

// Recovery methods
enum slam_recovery_method { spin, backtrack };

void action_upon_panic(Drone& drone);
void action_upon_future_col(Drone& drone);
bool action_upon_slam_loss(Drone& drone, slam_recovery_method slm...);

// Functions to manipulate and follow trajectories
typedef trajectory_msgs::MultiDOFJointTrajectoryPoint multiDOFpoint;
typedef std::deque<multiDOFpoint> trajectory_t;

trajectory_t follow_trajectory(Drone& drone, trajectory_t& trajectory, float time = 0.5);
trajectory_t append_trajectory(trajectory_t first, trajectory_t second);

// Spinning commands
void spin_around(Drone &drone);
void scan_around(Drone &drone, int angle);
void spin_slowly(Drone &drone, int n_pies=20);

// Utility functions
float distance(float x, float y, float z);

#endif

