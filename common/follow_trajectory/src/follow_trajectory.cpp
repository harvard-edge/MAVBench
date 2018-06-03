#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "common.h"
#include "Drone.h"
#include <std_msgs/Bool.h>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/future_collision.h>

using namespace std;

// Trajectories
trajectory_t trajectory;
trajectory_t reverse_trajectory;
bool fly_backward = false;

// Messages from other nodes
bool slam_lost = false;
ros::Time future_collision_time{0}; // The moment in time when the drone should stop because of an upcoming collision
int future_collision_seq = 0;
int trajectory_seq = 0;

// Parameters
float g_v_max;
double g_grace_period = 0; // How much time the drone will wait for a new path to be calculated when a collision is near, before pumping the breaks
double g_time_to_come_to_full_stop = 0;
double g_fly_trajectory_time_out;
long long g_planning_time_including_ros_overhead_acc  = 0;
float g_max_yaw_rate;
float g_max_yaw_rate_during_flight;
bool g_trajectory_done = false;
bool g_got_new_trajectory = false;

// Profiling
std::string g_supervisor_mailbox; //file to write to when completed
float g_localization_status = 1.0;
long long g_rcv_traj_to_follow_traj_acc_t = 0;
bool CLCT_DATA, DEBUG;
int g_follow_ctr = 0;
long long g_img_to_follow_acc = 0;
ros::Time g_msg_time_stamp;
long long g_pt_cld_to_futurCol_commun_acc = 0;
int g_traj_ctr = 0; 
ros::Time g_recieved_traj_t;
double g_max_velocity_reached = 0;

void log_data_before_shutting_down()
{
    std::cout << "\n\nMax velocity reached by drone: " << g_max_velocity_reached << "\n" << std::endl;

    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profiling_data_srv_inst.request.key = "localization_status";
    profiling_data_srv_inst.request.value = g_localization_status;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "img_to_follow_traj_commun_t";
    profiling_data_srv_inst.request.value = (((double)g_pt_cld_to_futurCol_commun_acc)/1e9)/g_traj_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "traj_rcv_to_follow";
    profiling_data_srv_inst.request.value = (((double)g_rcv_traj_to_follow_traj_acc_t)/1e9)/g_follow_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "image_to_follow_time";
    profiling_data_srv_inst.request.value = (((double)g_img_to_follow_acc)/1e9)/g_follow_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "max_velocity_reached";
    profiling_data_srv_inst.request.value = g_max_velocity_reached;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
}

void future_collision_callback(const mavbench_msgs::future_collision::ConstPtr& msg) {
    if (msg->future_collision_seq > future_collision_seq) {
        future_collision_seq = msg->future_collision_seq;

        if (g_grace_period+g_time_to_come_to_full_stop < msg->time_to_collision)
            future_collision_time = ros::Time::now() + ros::Duration(g_grace_period);
        else
            future_collision_time = ros::Time::now();
    }
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}


template<class P1, class P2>
trajectory_t straight_line_trajectory(P1 start, P2 end, double v)
{
    trajectory_t result;

    const double dt = 0.5;

    double correction_in_x = end.x - start.x;
    double correction_in_y = end.y - start.y;
    double correction_in_z = end.z - start.z;

    double correction_distance = distance(correction_in_x, correction_in_y, correction_in_z);
    double correction_time = correction_distance / v;

    double disc = std::min((dt * v) / correction_distance, 1.0); // The proportion of the correction_distance taken up by each g_dt time step

    double vx = correction_in_x / correction_time;
    double vy = correction_in_y / correction_time;
    double vz = correction_in_z / correction_time;

    double yaw = yawFromVelocity(vx, vy);

    for (double it = 0; it <= 1.0; it += disc) {
        multidofpoint p;

        p.x = start.x + it*correction_in_x;
        p.y = start.y + it*correction_in_y;
        p.z = start.z + it*correction_in_z;

        p.vx = vx;
        p.vy = vy;
        p.vz = vz;

        p.yaw = yaw;
        p.blocking_yaw = false;

        p.duration = dt;

        correction_path.push_back(p);
    }
}


void callback_trajectory(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg, Drone * drone)
{
    // Check for trajectories that arrive out of order
    if (msg->trajectory_seq < trajectory_seq) {
        ROS_ERROR("follow_trajectory: Trajectories arrived out of order! New seq: %d, old seq: %d", msg->trajectory_seq, trajectory_seq);
        return;
    } else
        trajectory_seq = msg->trajectory_seq;

    // Check for trajectories that are not updated to the latest collision detection
    if (msg->future_collision_seq < future_collision_seq) {
        ROS_ERROR("Proposed trajectory does not consider latest detected collision");
        return;
    } else {
        future_collision_seq = msg->future_collision_seq;
        future_collision_time = ros::Time(0);
    }

    trajectory_t new_trajectory = create_trajectory_from_msg(*msg);

    if (msg->reverse) {
        fly_backward = true;
    } else if (trajectory.empty() && !new_trajectory.empty()) {
        // Add drift correction if the drone is currently idling (because it will float around while idling)
        trajectory_t idling_correction_traj = straight_line_trajectory(drone->position(), new_trajectory.front(), 1.0);
        trajectory = append_trajectory(idling_correction_traj, new_trajectory);
        fly_backward = false;
    } else {
        trajectory = new_trajectory;
        fly_backward = false;
    }

    g_got_new_trajectory = true;

    // Profiling
    if (CLCT_DATA){
        g_recieved_traj_t = ros::Time::now();
        g_msg_time_stamp = msg->header.stamp;
        if (g_msg_time_stamp.sec != 0) {
            g_pt_cld_to_futurCol_commun_acc += (ros::Time::now() - msg->header.stamp).toSec()*1e9;
            g_traj_ctr++;
        } 
    }
}


bool trajectory_done(const trajectory_t& trajectory) {
    trajectory.size() == 0;
    g_trajectory_done = (trajectory.size() == 0);
    return g_trajectory_done;
}


void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        signal_supervisor(g_supervisor_mailbox, "kill"); 
        ros::shutdown();
    }
    exit(0);
}


void initialize_global_params() {
    if(!ros::param::get("v_max", g_v_max))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory vmax not provided");
        exit(-1);
    }

    if(!ros::param::get("max_yaw_rate",g_max_yaw_rate))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate not provided");
        exit(-1);
    }

    if(!ros::param::get("max_yaw_rate_during_flight",g_max_yaw_rate_during_flight))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate_during_flight not provided");
        exit(-1);
    }

    if(!ros::param::get("CLCT_DATA",CLCT_DATA))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory CLCT_DATA not provided");
        exit(-1);
    }
    if(!ros::param::get("DEBUG",DEBUG))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory DEBUG not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/fly_trajectory_time_out", g_fly_trajectory_time_out)){
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! fly_trajectory_time_out is not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/grace_period", g_grace_period)) {
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! grace_period is not provided");
        exit(-1);
    }

    double a_max;
    if(!ros::param::get("a_max", a_max))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory amax not provided");
        exit(-1);
    }

    g_time_to_come_to_full_stop = g_v_max / a_max;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_trajectory", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandlerPrivate);

    initialize_global_params();

    // Initialize the drone
    std::string localization_method;
    std::string ip_addr;
    const uint16_t port = 41451;

    ros::param::get("/follow_trajectory/ip_addr", ip_addr);
    ros::param::get("/follow_trajectory/localization_method", localization_method);
    Drone drone(ip_addr.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight);

    // Initialize publishers and subscribers
    ros::Publisher next_steps_pub = n.advertise<mavbench_msgs::multiDOFtrajectory>("/next_steps", 1);

    ros::Subscriber slam_lost_sub = n.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
    ros::Subscriber col_coming_sub = n.subscribe<mavbench_msgs::future_collision>("/col_coming", 1, future_collision_callback);
    // ros::Subscriber traj_sub = n.subscribe<mavbench_msgs::multiDOFtrajectory>("normal_traj", 1, callback_trajectory);
    ros::Subscriber traj_sub = n.subscribe<mavbench_msgs::multiDOFtrajectory>("multidoftraj", 1, boost::bind(callback_trajectory, _1, &drone));

    // Begin execution loop
    bool app_started = false;  //decides when the first planning has occured
                               //this allows us to activate all the
                               //functionaliy in follow_trajectory accordingly

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();

        if (trajectory.size() > 0) {
            app_started = true;
        }

        if (app_started) {
            // Profiling
            if (CLCT_DATA) {
                if (g_got_new_trajectory) {
                    g_rcv_traj_to_follow_traj_acc_t +=
                        (ros::Time::now() - g_recieved_traj_t).toSec()*1e9;
                    if (g_msg_time_stamp.sec != 0) {
                        g_img_to_follow_acc += (ros::Time::now() - g_msg_time_stamp).toSec()*1e9;
                        g_follow_ctr++; 
                    }
                    if (DEBUG) {
                        //ROS_INFO_STREAM("follow_traj_cb to  func" << g_rcv_traj_to_follow_traj_t);
                        //ROS_INFO_STREAM("whatevs------- " << g_img_to_follow_acc);
                    }
                }
            }
        }

        // Figure out which direction we will fly in
        trajectory_t * forward_traj;
        trajectory_t * rev_traj;

        yaw_strategy_t yaw_strategy = follow_yaw;
        float max_velocity = g_v_max;

        if (!fly_backward) {
            forward_traj = &trajectory;
            rev_traj = &reverse_trajectory;
        } else {
            forward_traj = &reverse_trajectory;
            rev_traj = &trajectory;

            yaw_strategy = face_backward;
            max_velocity = 1;
        }

        double max_velocity_reached = follow_trajectory(drone, forward_traj,
                rev_traj, yaw_strategy, true, max_velocity,
                g_fly_trajectory_time_out);

        if (max_velocity_reached > g_max_velocity_reached)
            g_max_velocity_reached = max_velocity_reached;

        // Publish the remainder of the trajectory
        mavbench_msgs::multiDOFtrajectory trajectory_msg = create_trajectory_msg(*forward_traj);
        trajectory_msg.future_collision_seq = future_collision_seq;
        trajectory_msg.trajectory_seq = trajectory_seq;
        trajectory_msg.reverse = fly_backward;

        next_steps_pub.publish(trajectory_msg);

        if (slam_lost){
            ROS_INFO_STREAM("slam loss");
            log_data_before_shutting_down();
            g_localization_status = 0;
            signal_supervisor(g_supervisor_mailbox, "kill");
            ros::shutdown();
        } else if (future_collision_time != ros::Time(0) && ros::Time::now() >= future_collision_time) {
            // Stop the drone if we haven't been able to come up with a new plan in our budgetted time
            ROS_WARN("Motion planner took too long to propose a new path, so the drone is being stopped!");
            drone.fly_velocity(0, 0, 0);
            trajectory.clear();
            future_collision_time = ros::Time(0);
        } else if (trajectory_done(*forward_traj)){
            loop_rate.sleep();
        }

        g_got_new_trajectory = false;
    }

    return 0;
}

