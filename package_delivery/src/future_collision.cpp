#include "ros/ros.h"
#include <cmath>
#include "std_msgs/Bool.h"
#include "visualization_msgs/Marker.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"

// Octomap specific includes
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

// Global variables
octomap::OcTree * octree = nullptr;
trajectory_msgs::MultiDOFJointTrajectory traj;
double drone_height__global;
double drone_radius__global;
visualization_msgs::Marker marker;

struct coord {
    double x, y , z;
};

template <class T>
bool collision(octomap::OcTree * octree, const T& n1, const T& n2)
{
	const double pi = 3.14159265359;

	// The drone is modeled as a cylinder.
	// Angles are in radians and lengths are in meters.

    static double height = [] () {
        double h;
        h = drone_height__global; 
        //ros::param::get("/motion_planner/drone_height", h);
        return h;
    } ();

    static double radius = [] () {
        double r;
        r = drone_radius__global * 0.75; 
        //ros::param::get("/motion_planner/drone_radius", r);
        return r;
    } ();

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
                    // ROS_WARN("future collision detected on %f, %f ,%f", n1.x, n1.y, n1.z);
                    
                    marker.header.stamp = ros::Time();
                    marker.pose.position.x = n1.x;
                    marker.pose.position.y = n1.y;
                    marker.pose.position.z = n1.z;
                    
					return true;
                }
			}
		}
	}

	return false;
}


void pull_octomap(const octomap_msgs::Octomap& msg)
{
    if (octree != nullptr) {
        delete octree;
    }

    if (msg.binary) {
        ROS_ERROR("Octomap is not full.");
    }

	octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(msg);
	octree = dynamic_cast<octomap::OcTree*> (tree);

    if (octree == nullptr) {
        ROS_ERROR("Octree could not be pulled.");
    }
}


void pull_traj(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
    traj = *msg;
}


bool check_for_collisions()
{
    if (octree == nullptr || traj.points.size() < 1) {
        return false;
    }

    bool col = false;

    for (int i = 0; !col && i < traj.points.size() - 1; ++i) {
        auto& pos1 = traj.points[i].transforms[0].translation;
        auto& pos2 = traj.points[i+1].transforms[0].translation;

        if (collision(octree, pos1, pos2))
            col = true;
    }

    return col;
}


void future_collision_initialize_params() {
    ros::param::get("/motion_planner/drone_radius", drone_radius__global);
    ros::param::get("/motion_planner/drone_height", drone_height__global);

    marker.header.frame_id = "fcu";
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    // only if using a MESH_RESOURCE marker type:
    // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}


int main(int argc, char** argv)
{
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    std_msgs::Bool col_msg;
    ros::init(argc, argv, "future_collision");
    ros::NodeHandle nh;

    ros::Subscriber octomap_sub = nh.subscribe("octomap_full", 1, pull_octomap);
    ros::Subscriber traj_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("multidoftraj", 1, pull_traj);
    ros::Publisher collision_publisher = nh.advertise<std_msgs::Bool>("future_col_topic", 1);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "collision_marker", 1);
    
    
    //----------------------------------------------------------------- 
    // *** F:DN BODY
    //----------------------------------------------------------------- 
    future_collision_initialize_params(); 
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        col_msg.data = check_for_collisions();
        collision_publisher.publish(col_msg);
        vis_pub.publish( marker );

        ros::spinOnce();
        loop_rate.sleep();
  }
}

