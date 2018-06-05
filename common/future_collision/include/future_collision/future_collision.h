#ifndef MAVBENCH_FUTURE_COLLISION_H
#define MAVBENCH_FUTURE_COLLISION_H

#include "ros/ros.h"

// Standard headers
#include <string>
#include <tuple>

// Octomap specific headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// MAVBench headers
#include "Drone.h"
#include "common.h"
#include "timer.h"
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/future_collision.h>

// Octomap server headers. This is only needed for profiling services
// TODO: Get rid of this dependency
#include <octomap_server/OctomapServer.h>

class FutureCollisionChecker {
public:
    FutureCollisionChecker(octomap::OcTree * octree_, Drone * drone_) :
        nh("~"),
        octree(octree_),
        drone(drone_)
    {
        future_collision_initialize_params();

        // Create a new callback queue
        nh.setCallbackQueue(&callback_queue);

        // Topics
        next_steps_sub = nh.subscribe<mavbench_msgs::multiDOFtrajectory>("/next_steps", 1, &FutureCollisionChecker::pull_traj, this);
        col_coming_pub = nh.advertise<mavbench_msgs::future_collision>("/col_coming", 1);

        // Create new Drone object
        // drone = new Drone(ip_addr__global.c_str(), port, localization_method);
    }

    void spinOnce();
    void log_data_before_shutting_down();

    // TODO: Get rid of this function
    void setOctomapServer(octomap_server::OctomapServer * server_)
    {
        server = server_;
    }

private:
    std::tuple <bool, double> check_for_collisions(Drone& drone);
    bool collision(const octomap::OcTree * octree, const multiDOFpoint& n1, const multiDOFpoint& n2) const;
    void pull_traj(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg);
    void future_collision_initialize_params();
    void stop_drone();

private:
    ros::NodeHandle nh;
    ros::CallbackQueue callback_queue;
    ros::Publisher col_coming_pub;
    ros::Subscriber next_steps_sub;

    Drone * drone = nullptr;

    int future_collision_seq_id = 0;
    const octomap::OcTree * octree = nullptr;
    trajectory_t traj;
    int traj_future_collision_seq_id = 0;

    // Parameters
    std::string ip_addr__global;
    std::string localization_method;
    double drone_height__global;
    double drone_radius__global;
    double grace_period__global;

    // Profiling variables
    const octomap_server::OctomapServer * server = nullptr; // TODO: get rid of this dependency
    ros::Time start_hook_chk_col_t, end_hook_chk_col_t;                                          
    long long g_checking_collision_kernel_acc = 0;
    ros::Time g_checking_collision_t;
    long long g_future_collision_main_loop = 0;
    int g_check_collision_ctr = 0;
    double g_distance_to_collision_first_realized = 0;
    bool CLCT_DATA = false;
    bool DEBUG = false;
    ros::Time g_pt_cloud_header; //this is used to figure out the octomap msg that 
                              //collision was detected in
    long long g_pt_cloud_future_collision_acc = 0;
    int g_octomap_rcv_ctr = 0;
    ros::Duration g_pt_cloud_to_future_collision_t; 
    int g_main_loop_ctr = 0;
    long long g_accumulate_loop_time = 0; //it is in ms
    long long g_pt_cld_to_octomap_commun_olverhead_acc = 0;
    long long octomap_integration_acc = 0;
    int octomap_ctr = 0;
};

#endif

