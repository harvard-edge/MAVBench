#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <limits>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Vector3.h>

#include "Drone.h"


typedef struct KeyValuePair{
    std::string key;
    double value;
    KeyValuePair(std::string key, double value): key(key), value(value){
    }

} KeyValuePairStruct;

typedef struct stats{
    long long accumulate;
    long long accumulate_sqr;
    int ctr; 
     
    stats(){
         this->accumulate = 0;
         this->accumulate_sqr = 0;
         this->ctr = 0;
    }
    
    stats(long long accumulate, long long accumulate_sqr, int ctr): 
                 accumulate(accumulate), accumulate_sqr(accumulate_sqr), 
                 ctr(ctr) {
    }

	// accumulate values  
	void acc(long long accumulate, long long accumulate_sqr){
        this->accumulate += accumulate;
        this->accumulate_sqr += accumulate_sqr;
        this->ctr += 1;
    }
} statsStruct;


void sigIntHandler(int sig);

void signal_supervisor(std::string file_to_write_to, std::string);
// Stats functions
void update_stats_file(const std::string& stats_file__addr, const std::string& content);
//void output_flight_summary(msr::airlib::FlightStats init, msr::airlib::FlightStats end, std::string mission_status, float coverage, double cpu_compute_enenrgy, double gpu_compute_enenrgy, const std::string& fname);


// Functions and classes to manipulate and follow trajectories
struct multiDOFpoint {
    double x, y, z;
    double vx, vy, vz;
    double yaw;
    double duration;
};
typedef std::deque<multiDOFpoint> trajectory_t;
enum yaw_strategy_t { ignore_yaw, face_forward, face_backward, follow_yaw };

trajectory_t create_trajectory(const trajectory_msgs::MultiDOFJointTrajectory&, bool face_forward = false);
trajectory_msgs::MultiDOFJointTrajectory create_trajectory_msg(const trajectory_t&);

void follow_trajectory(Drone& drone, trajectory_t * traj,
        trajectory_t * reverse_traj,
        yaw_strategy_t yaw_strategy = ignore_yaw,
        bool check_position = true,
        float max_speed = std::numeric_limits<double>::infinity(),
        float time = 0.5);


// Recovery methods
trajectory_t create_panic_trajectory(Drone& drone, const geometry_msgs::Vector3& panic_dir);
trajectory_t create_future_col_trajectory(const trajectory_t& normal_traj, double stopping_distance);
trajectory_t create_slam_loss_trajectory(Drone& drone, trajectory_t& normal_traj, const trajectory_t& rev_normal_traj);

bool reset_slam(Drone& drone, const std::string& topic);


// Spinning commands
void spin_around(Drone &drone);
void scan_around(Drone &drone, int angle);


// Utility functions
float distance(float x, float y, float z);
float yawFromQuat(geometry_msgs::Quaternion q);
void waitForLocalization(std::string method);

#endif

