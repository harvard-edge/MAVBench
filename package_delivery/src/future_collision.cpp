#include "ros/ros.h"

// Standard headers
#include <chrono>
#include <string>
#include <cmath>

// ROS message headers
#include "std_msgs/Bool.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"

// Octomap specific headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

// MAVBench headers
#include "Drone.h"
#include "timer.h"

// Typedefs
typedef trajectory_msgs::MultiDOFJointTrajectory traj_msg_t;
typedef std::chrono::system_clock sys_clock;
typedef std::chrono::time_point<sys_clock> sys_clock_time_point;

// Global variables
octomap::OcTree * octree = nullptr;
traj_msg_t traj;
std::string ip_addr__global;
std::string localization_method;
double drone_height__global;
double drone_radius__global;

template <class T>
bool collision(octomap::OcTree * octree, const T& n1, const T& n2)
{
	const double pi = 3.14159265359;

    const double height = drone_height__global; 
    const double radius = drone_radius__global; 

	const double angle_step = pi/4;
	const double radius_step = radius/3;
	const double height_step = height/2;

	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;
	double dz = n2.z - n1.z;

	double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

	octomap::point3d direction(dx, dy, dz);
	octomap::point3d end;

	for (double h = -height/2; h <= height/2; h += height_step) {
		for (double r = 0; r <= radius; r += radius_step) {
			for (double a = 0; a <= pi*2; a += angle_step) {
				octomap::point3d start(n1.x + r*std::cos(a), n1.y + r*std::sin(a), n1.z + h);

				if (octree->castRay(start, direction, end, true, distance)) {
					return true;
				}
			}
		}
	}

	return false;
}


template <class T>
double dist_to_collision(Drone& drone, const T& col_pos) {
    auto drone_pos = drone.position();

	double dx = drone_pos.x - col_pos.x;
	double dy = drone_pos.y - col_pos.y;
	double dz = drone_pos.z - col_pos.z;

	return std::sqrt(dx*dx + dy*dy + dz*dz);
}


void pull_octomap(const octomap_msgs::Octomap& msg)
{
    RESET_TIMER();
    if (octree != nullptr) {
        delete octree;
    }

	octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(msg);
	octree = dynamic_cast<octomap::OcTree*> (tree);

    if (octree == nullptr) {
        ROS_ERROR("Octree could not be pulled.");
    }

    LOG_ELAPSED(future_collision_pull);
}


void pull_traj(const traj_msg_t::ConstPtr& msg)
{
    traj = *msg;
}


bool check_for_collisions(Drone& drone, sys_clock_time_point& time_to_warn)
{
    RESET_TIMER();

    const double min_dist_from_collision = 5.0;
    const std::chrono::milliseconds grace_period(2000);

    if (octree == nullptr || traj.points.size() < 1) {
        return false;
    }

    bool col = false;

    for (int i = 0; i < traj.points.size() - 1; ++i) {
        auto& pos1 = traj.points[i].transforms[0].translation;
        auto& pos2 = traj.points[i+1].transforms[0].translation;

        if (collision(octree, pos1, pos2)) {
            col = true;

            // Check whether the drone is very close to the point of collision
            auto now = sys_clock::now();
            if (dist_to_collision(drone, pos1) < min_dist_from_collision)
                time_to_warn = now;
            // Otherwise, give the drone a grace period to continue along its
            // path. Don't update the time_to_warn if it's already been set to
            // some time in the future
            else if (now > time_to_warn)
                time_to_warn = now + grace_period;

            break;
        }
    }

    if (!col)
        time_to_warn = sys_clock_time_point::min();

    LOG_ELAPSED(future_collision);
    return col;
}


void future_collision_initialize_params()
{
    ros::param::get("/motion_planner/drone_radius", drone_radius__global);
    ros::param::get("/motion_planner/drone_height", drone_height__global);

    if(!ros::param::get("/package_delivery/ip_addr",ip_addr__global)){
        ROS_FATAL("Could not start exploration. IP address parameter missing!");
        return;
    }

    if(!ros::param::get("/package_delivery/localization_method",localization_method)){
        ROS_FATAL("Could not start exploration. Localization parameter missing!");
        return; 
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "future_collision");
    ros::NodeHandle nh;

    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    future_collision_initialize_params(); 

    bool collision_imminent = false;
    auto time_to_warn = sys_clock_time_point::min();

    std_msgs::Bool col_msg;

    ros::Subscriber octomap_sub = nh.subscribe("octomap_binary", 1, pull_octomap);
    ros::Subscriber traj_sub = nh.subscribe<traj_msg_t>("multidoftraj", 1, pull_traj);

    ros::Publisher collision_publisher = nh.advertise<std_msgs::Bool>("future_col_topic", 1);

    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);

    
    //----------------------------------------------------------------- 
    // *** F:DN BODY
    //----------------------------------------------------------------- 

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        collision_imminent = check_for_collisions(drone, time_to_warn);

        if (collision_imminent) {
            auto now = sys_clock::now();
            if (now > time_to_warn) {
                col_msg.data = true;
                collision_publisher.publish(col_msg);
            }
        } else {
            col_msg.data = false;
            collision_publisher.publish(col_msg);
        }

        loop_rate.sleep();
    }
}

