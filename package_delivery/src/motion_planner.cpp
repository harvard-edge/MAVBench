#include <ros/ros.h>

// Standard headers
#include <cstdlib>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include <iostream>
#include <thread>
#include <functional>
#include <limits>
#include <signal.h>

// My headers
#include "common.h"
#include "graph.h"
#include "global_planner.h"
#include "package_delivery/get_trajectory.h"
#include "timer.h"
#include <profile_manager/profiling_data_srv.h>

// Misc messages
#include <geometry_msgs/Point.h>

// Octomap specific headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_world/octomap_world.h>

// TF specific headers
#include <tf/transform_datatypes.h>

// Pointcloud headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Trajectory smoothening headers
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>

// OMPL specific headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// Type-defs
using piecewise_trajectory = std::vector<graph::node>;
using smooth_trajectory = mav_trajectory_generation::Trajectory;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;


// Parameters
graph::node_id start_id,  goal_id;
std::string motion_planning_core_str;
bool DEBUG__global;
double drone_height__global;
double drone_radius__global;
double rrt_step_size__global;
int rrt_bias__global;
double x__low_bound__global, x__high_bound__global;
double y__low_bound__global, y__high_bound__global;
double z__low_bound__global, z__high_bound__global;
int nodes_to_add_to_roadmap__global;
double max_dist_to_connect_at__global;
double sampling_interval__global;
double v_max__global, a_max__global;
int max_roadmap_size__global;
std::function<piecewise_trajectory (geometry_msgs::Point, geometry_msgs::Point, int, int , int, octomap::OcTree *)> motion_planning_core;
long long g_path_computation_without_OM_PULL_time_acc = 0;
static int g_number_of_planning = 0 ;

//*** F:DN global variables
octomap::OcTree * octree = nullptr;
trajectory_msgs::MultiDOFJointTrajectory traj_topic;
ros::ServiceClient octo_client;

// The following block of global variables only exist for debugging purposes
visualization_msgs::MarkerArray smooth_traj_markers;
visualization_msgs::MarkerArray piecewise_traj_markers;
octomap_msgs::Octomap omp;
PointCloud::Ptr pcl_ptr{new pcl::PointCloud<pcl::PointXYZ>};
visualization_msgs::Marker graph_conn_list;
ros::Publisher graph_conn_pub;
// #define INFLATE

// *** F:DN calculating the distance between two nodes in the graph.
double dist(const graph::node& n1, const graph::node& n2);


// *** F:DN Checks whether a cell in the occupancy grid is occupied.
bool occupied(octomap::OcTree * octree, double x, double y, double z);


// *** F:DN Checks whether a cell in the occupancy grid is known.
bool known(octomap::OcTree * octree, double x, double y, double z);


// *** F:DN Checks whether there is a collision between two nodes in the occupancy grid.
bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr = nullptr);


// *** F:DN Checks whether a cell in the occupancy grid is occupied.
bool out_of_bounds(const graph::node& pos);


// *** F:DN find all neighbours within "max_dist" meters of node
std::vector<graph::node_id> nodes_in_radius(/*const*/ graph& g, graph::node_id n, double max_dist, octomap::OcTree * octree);


// *** F:DN Request an octomap from the octomap_server
void request_octomap();


// *** F:DN Clear area in bounding box
void clear_octomap_bbx(const graph::node& pos);


// *** F:DN Generate and inflate an octomap from a message
void generate_octomap(const octomap_msgs::Octomap& msg);


// *** F:DN Initializes the PRM.
graph create_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree *octree, graph::node_id &start_id, graph::node_id &goal_id);


piecewise_trajectory lawn_mower(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);

// *** F:DN Increases the density of the PRM.
void extend_PRM(graph &roadmap, octomap::OcTree * octree);


// ***F:DN Use the PRM sampling method to find a piecewise path
piecewise_trajectory PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);


// ***F:DN Use the RRT sampling method to find a piecewise path
piecewise_trajectory RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);


// ***F:DN Use the RRT sampling method from OMPL to find a piecewise path
piecewise_trajectory OMPL_RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);


// ***F:DN Use bi-directonal RRT from OMPL to find a piecewise path
piecewise_trajectory OMPL_RRTConnect(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);


// ***F:DN Use the PRM sampling method from OMPL to find a piecewise path
piecewise_trajectory OMPL_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);


// *** F:DN Optimize and smoothen a piecewise path without causing any new collisions.
smooth_trajectory smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree);


// ***F:DN Build the response to the service from the smooth_path
void create_response(package_delivery::get_trajectory::Response &res, smooth_trajectory& smooth_path);


// ***F:DN Temporary debugging function that publishes the roadmap (without the connections) for visualization.
void publish_graph(graph& g);


// ***F:DN Post-process piecewise path to optimize it.
void postprocess(piecewise_trajectory& path);


//*** F:DN getting the smoothened trajectory
bool get_trajectory_fun(package_delivery::get_trajectory::Request &req, package_delivery::get_trajectory::Response &res)
{
    x__low_bound__global = std::min(x__low_bound__global, req.start.x);
    x__high_bound__global = std::max(x__high_bound__global, req.start.x);
    y__low_bound__global = std::min(y__low_bound__global, req.start.y);
    y__high_bound__global = std::max(y__high_bound__global, req.start.y);
    z__low_bound__global = std::min(z__low_bound__global, req.start.z);
    z__high_bound__global = std::max(z__high_bound__global, req.start.z);


    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
	piecewise_trajectory piecewise_path;
	smooth_trajectory smooth_path;

    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 

    request_octomap();
    if (octree == nullptr) {
    	ROS_ERROR("Octomap is not available.");
    	return false;
    }
    clear_octomap_bbx({req.start.x, req.start.y, req.start.z});

    // octomap_msgs::binaryMapToMsg(*octree, omp);
    //octree->writeBinary("/home/ubuntu/octomap.bt");


    auto loop_start_t = ros::Time::now();
    piecewise_path = motion_planning_core(req.start, req.goal, req.width, req.length ,req.n_pts_per_dir, octree);
    //piecewise_path = motion_planning_core(req.start, req.goal, octree);

    if (piecewise_path.size() == 0) {
        ROS_ERROR("Empty path returned");
        return false;
    }

    //ROS_INFO("Path size: %u. Now post-processing...", piecewise_path.size());

    if (motion_planning_core_str != "lawn_mower") {
        postprocess(piecewise_path);
    }

    //ROS_INFO("Path size: %u. Now smoothening...", piecewise_path.size());

    // Smoothen the path and build the multiDOFtrajectory response
    smooth_path = smoothen_the_shortest_path(piecewise_path, octree);
	
    create_response(res, smooth_path);

    // Publish the trajectory (for debugging purposes)
    traj_topic = res.multiDOFtrajectory;

    auto loop_end_t = ros::Time::now(); 
    g_path_computation_without_OM_PULL_time_acc += (((loop_end_t - loop_start_t).toSec())*1e9);
    g_number_of_planning++; 
    //ROS_INFO_STREAM("planning takes"<<(loop_end_t - loop_start_t).toSec());
    return true;
}


// *** F:DN initializing all the global variables 
void motion_planning_initialize_params() {

    ros::param::get("motion_planner/max_roadmap_size", max_roadmap_size__global);
    ros::param::get("/motion_planner/sampling_interval", sampling_interval__global);
    ros::param::get("/motion_planner/rrt_step_size", rrt_step_size__global);
    ros::param::get("/motion_planner/rrt_bias", rrt_bias__global);
    ros::param::get("/motion_planner/x_dist_to_sample_from__low_bound", x__low_bound__global);
    ros::param::get("/motion_planner/x_dist_to_sample_from__high_bound", x__high_bound__global);

    ros::param::get("/motion_planner/y_dist_to_sample_from__low_bound", y__low_bound__global);
    ros::param::get("/motion_planner/y_dist_to_sample_from__high_bound", y__high_bound__global);
    ros::param::get("/motion_planner/z_dist_to_sample_from__low_bound", z__low_bound__global);
    ros::param::get("/motion_planner/z_dist_to_sample_from__high_bound", z__high_bound__global);
    ros::param::get("/motion_planner/nodes_to_add_to_roadmap", nodes_to_add_to_roadmap__global);
    ros::param::get("/motion_planner/max_dist_to_connect_at", max_dist_to_connect_at__global);

    ros::param::get("/motion_planner/drone_radius", drone_radius__global);
    ros::param::get("/motion_planner/drone_height", drone_height__global);
    ros::param::get("/motion_planner/v_max", v_max__global);
    ros::param::get("/motion_planner/a_max", a_max__global);
    ros::param::get("ros_DEBUG", DEBUG__global);
    //std::cout<<"max_dist_to_"<<max_dist_to_connect_at__global<<std::endl;
    
    ros::param::get("/motion_planner/motion_planning_core", motion_planning_core_str);
    if (motion_planning_core_str == "PRM")
        motion_planning_core = PRM;
    else if (motion_planning_core_str == "RRT")
        motion_planning_core = RRT;
    else if (motion_planning_core_str == "lawn_mower")
        motion_planning_core = lawn_mower;
    else if (motion_planning_core_str == "OMPL-RRT")
        motion_planning_core = OMPL_RRT;
    else if (motion_planning_core_str == "OMPL-RRTConnect")
        motion_planning_core = OMPL_RRTConnect;
    else if (motion_planning_core_str == "OMPL-PRM")
        motion_planning_core = OMPL_PRM;
    else{
        std::cout<<"This motion planning type is not defined"<<std::endl;
        exit(0);
    }

}

void log_data_before_shutting_down(){
    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;

    profiling_data_srv_inst.request.key = "number_of_plannings";
    profiling_data_srv_inst.request.value = g_number_of_planning;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "g_path_computation_without_OM_PULL_time_acc";
    profiling_data_srv_inst.request.value = g_path_computation_without_OM_PULL_time_acc;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "g_path_computation_time_avg";
    profiling_data_srv_inst.request.value = g_path_computation_without_OM_PULL_time_acc/g_number_of_planning;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

}

void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        ros::shutdown();
    }
    exit(0);
}

int main(int argc, char ** argv)
{
    //----------------------------------------------------------------- 
    // *** F:DN variables	
    //----------------------------------------------------------------- 
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;
    motion_planning_initialize_params();
    signal(SIGINT, sigIntHandlerPrivate);


    // *** F:DN topics and services
    ros::ServiceServer service = nh.advertiseService("get_trajectory_srv", get_trajectory_fun);
    ros::Publisher smooth_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);
    ros::Publisher piecewise_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("multidoftraj", 1);
    ros::Publisher octo_pub = nh.advertise<octomap_msgs::Octomap>("omap", 1);
    ros::Publisher pcl_pub = nh.advertise<PointCloud> ("graph", 1);
    graph_conn_pub = nh.advertise<visualization_msgs::Marker>("graph_conns", 100);
    // ros::Subscriber octomap_sub = nh.subscribe("octomap_full", 1, generate_octomap);
    octo_client = nh.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");
	
    pcl_ptr->header.frame_id = graph_conn_list.header.frame_id = "world";
    graph_conn_list.type = visualization_msgs::Marker::LINE_LIST;
    graph_conn_list.action = visualization_msgs::Marker::ADD;
    graph_conn_list.scale.x = 0.1;
    graph_conn_list.pose.orientation.w = 1;
    graph_conn_list.color.r = 1;
    graph_conn_list.color.a = 1;

    /* //TODO place a sanity check making sure that panic distance is smaller than halo
    float panic_distance = ros::param::get("/panic_pcl/safe_distance",panic_distance);
    float  
    */

    

    //----------------------------------------------------------------- 
    // *** F:DN BODY
    //----------------------------------------------------------------- 
	ros::Rate pub_rate(5);
	while (ros::ok())
	{
        if (DEBUG__global) { //if debug, publish markers to be seen by rviz
            smooth_traj_vis_pub.publish(smooth_traj_markers);
            piecewise_traj_vis_pub.publish(piecewise_traj_markers);
            graph_conn_pub.publish(graph_conn_list);
            octo_pub.publish(omp);
            pcl_pub.publish(pcl_ptr);
        }
        traj_pub.publish(traj_topic);
		ros::spinOnce();
		pub_rate.sleep();
	}

    return 0;
}


double dist(const graph::node& n1, const graph::node& n2)
{
	return std::sqrt((n1.x-n2.x)*(n1.x-n2.x) + (n1.y-n2.y)*(n1.y-n2.y) + (n1.z-n2.z)*(n1.z-n2.z));
}


bool occupied(octomap::OcTree * octree, double x, double y, double z)
{
	const double OCC_THRESH = 0.5;

	octomap::OcTreeNode * otn = octree->search(x, y, z);

	return otn != nullptr && otn->getOccupancy() >= OCC_THRESH;
}


bool known(octomap::OcTree * octree, double x, double y, double z)
{
	return octree->search(x, y, z) != nullptr;
}


bool out_of_bounds(const graph::node& pos) {
    return (pos.x < x__low_bound__global
            || pos.x > x__high_bound__global
            || pos.y < y__low_bound__global
            || pos.y > y__high_bound__global
            || pos.z < z__low_bound__global
            || pos.z > z__high_bound__global);
}


#ifdef INFLATE
bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr)
{
    RESET_TIMER();
    // First, check if anything goes underground
    if (n1.z <= 0 || n2.z <= 0)
        return true;
            
	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;
	double dz = n2.z - n1.z;

	double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    octomap::point3d start(n1.x, n1.y, n1.z);
	octomap::point3d direction(dx, dy, dz);
	octomap::point3d end;

    bool collided = octree->castRay(start, direction, end, true, distance);

    if (end_ptr != nullptr && collided) {
        end_ptr->x = end.x();
        end_ptr->y = end.y();
        end_ptr->z = end.z();
    }

	LOG_ELAPSED(motion_planner);
	return collided;
}
#else
bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr)
{
    if (motion_planning_core_str == "lawn_mower") {
        return false;
    }
    
    
    RESET_TIMER();
    // First, check if anything goes underground
    if (n1.z <= 0 || n2.z <= 0)
        return true;
            
    const double pi = 3.14159265359;

	// The drone is modeled as a cylinder.
	// Angles are in radians and lengths are in meters.
    
    double height = drone_height__global; 
    double radius = drone_radius__global; 

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

                    if (end_ptr != nullptr) {
                        end_ptr->x = end.x();
                        end_ptr->y = end.y();
                        end_ptr->z = end.z();
                    }

					LOG_ELAPSED(motion_planner);
					return true;
                }
			}
		}
	}

	LOG_ELAPSED(motion_planner);
	return false;
}
#endif

std::vector<graph::node_id> nodes_in_radius(/*const*/ graph& g, graph::node_id n, double max_dist, octomap::OcTree * octree)
{
	auto node_ids = g.node_ids();
	node_ids.erase(n);
	std::vector<graph::node_id> result;

	for (const auto& n2 : node_ids) {
		if (dist(g.get_node(n), g.get_node(n2)) <= max_dist && !collision(octree, g.get_node(n), g.get_node(n2))) {
			result.push_back(n2);
		}
	}

	return result;
}


void request_octomap()
{
    octomap_msgs::GetOctomap srv;
    
    if (octo_client.call(srv))
        generate_octomap(srv.response.map);
    else
        ROS_ERROR("Octomap service request failed");
}


void clear_octomap_bbx(const graph::node& pos)
{
    if (occupied(octree, pos.x, pos.y, pos.z)) {
        ROS_WARN("Start is already occupied!");
    }

    // Free space around the start since is assumed to be open
    const double epsilon = 0.1;
    double r = drone_radius__global + epsilon;
    double h = drone_height__global + epsilon;

    octomap::point3d min(pos.x-r, pos.y-r, pos.z-h/2);
    octomap::point3d max(pos.x+r, pos.y+r, pos.z+h/2);

    double threshMin = octree->getClampingThresMin();
    for (auto it = octree->begin_leafs_bbx(min, max),
            end = octree->end_leafs_bbx(); it != end; ++it) {
        it->setLogOdds(octomap::logodds(threshMin));
    }
    octree->updateInnerOccupancy();
}


void generate_octomap(const octomap_msgs::Octomap& msg)
{
    RESET_TIMER();
    if (octree != nullptr) {
        delete octree;
    }

    ROS_INFO("Requesting octomap...");

#ifdef INFLATE
    // Inflate Octomap
    ROS_INFO("Inflating..");
    volumetric_mapping::OctomapWorld ocworld;
    ocworld.setOctomapFromMsg(msg);
    Eigen::Vector3d safety_radius(drone_radius__global, drone_radius__global,
            drone_radius__global);
    ocworld.inflateOccupied(safety_radius);

    // Convert inflated OctomapWorld to Octree
    octomap_msgs::Octomap inflated_msg;
    ocworld.getOctomapBinaryMsg(&inflated_msg);
	octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(inflated_msg);
#else
    octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(msg);
#endif
	octree = dynamic_cast<octomap::OcTree*> (tree);

    if (octree == nullptr) {
        ROS_ERROR("Octree could not be pulled.");
    }

    LOG_ELAPSED(motion_planner_pull);
}


graph create_lawnMower_path(geometry_msgs::Point start, int width, int length, int n_pts_per_dir, octomap::OcTree *octree, graph::node_id &start_id, graph::node_id &goal_id)

{
	
    // *** F:DN variables 
    graph roadmap;
	bool success = true;
    double x_step = double(length)/double(n_pts_per_dir);
    double y_step = double(width)/double(n_pts_per_dir);
    graph::node_id cur_node_id, prev_node_id;
    double x = start.x;
    double y = start.y;


	//ROS_INFO("starting piecewise_path");
	ROS_INFO("starting x,y is %f %f", x, y);
	//start_id = -1, goal_id = -2;
	
    //*** F:DN generate all the nodes
        for (int i = 0 ; i < n_pts_per_dir; i++) {
            for (int j = 0 ; j < n_pts_per_dir; j++) {

                ROS_INFO("%f %f", x, y);
                if (i==0 && j==0) {
                    graph::node_id cur_node_id = roadmap.add_node(
                            x, y, start.z);
                }
                else{
                    graph::node_id cur_node_id = roadmap.add_node(x, y, start.z);
                    roadmap.connect(cur_node_id, prev_node_id, 
                            dist(roadmap.get_node(cur_node_id), 
                                roadmap.get_node(prev_node_id)));
                }
                ROS_INFO("id is %d", int(cur_node_id)); 
                y +=y_step;
                prev_node_id = cur_node_id; 
            }
	   
            /*
            if ((i+1)  < n_pts_per_dir) { 
                    
                    ROS_INFO("%f %f", x, y);
		    x += (x_step/3); 
	            cur_node_id = roadmap.add_node(x, y, start.z);
		    roadmap.connect(cur_node_id, prev_node_id, 
				    dist(roadmap.get_node(cur_node_id), 
					    roadmap.get_node(prev_node_id)));
		    prev_node_id = cur_node_id; 
                    ROS_INFO("%f %f", x, y);
		    x += (x_step/3); 
		    cur_node_id = roadmap.add_node(x, y, start.z);
		    roadmap.connect(cur_node_id, prev_node_id, 
				    dist(roadmap.get_node(cur_node_id), 
					    roadmap.get_node(prev_node_id)));
		    prev_node_id = cur_node_id; 
		    x += (x_step/3);
	    } 
             */          
	    x += x_step;
	    y_step *= -1;
    }
   
    // ***F:DN returning back the the origin
    cur_node_id = roadmap.add_node(start.x, start.y, start.z);
    roadmap.connect(cur_node_id, prev_node_id, 
            dist(roadmap.get_node(cur_node_id), 
                roadmap.get_node(prev_node_id)));
        
    cout <<"road map"<<endl;
    cout<<roadmap;
	if (occupied(octree, start.x, start.y, start.z)) {
		ROS_ERROR("Start is already occupied!");
		success = false;
	}
    return roadmap;
}

graph create_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree *octree, graph::node_id &start_id, graph::node_id &goal_id)
{
	graph roadmap;
	bool success = true;

	ROS_INFO("starting piecewise_path");
	ROS_INFO("initialized roadmap");
	start_id = -1, goal_id = -2;
	graph::node s = {start.x, start.y, start.z, start_id};
	graph::node g = {goal.x, goal.y, goal.z, goal_id};

	ROS_INFO("Start %f %f %f", start.x, start.y, start.z);
	ROS_INFO("Goal  %f %f %f", goal.x, goal.y, goal.z);

	// Check whether the path is even possible.
	// The path is impossible if the start or end coordinates are in an occupied part of the octomap.
	
	if (occupied(octree, start.x, start.y, start.z)) {
		ROS_WARN("Start is already occupied!");
		// success = false;
	}

	if (occupied(octree, goal.x, goal.y, goal.z)) {
		ROS_ERROR("Goal is already occupied!");
		success	= false;
	}


	// If the path is believed to be possible, then add the start and end nodes.
	// Note: the path may still be impossible. Our earlier check is quite rudimentary.
	if (success) {
		roadmap.add_node(s);
		roadmap.add_node(g);

		// If possible, connect the start and end nodes directly.
		// Then, the shortest path will be a straight line between the start and goal.
		if (!collision(octree, s, g)) {
			ROS_INFO("No collision so connecting start and goal directly.");
			roadmap.connect(start_id, goal_id);
		}
	}
	return roadmap;
}


void extend_PRM(graph &roadmap, octomap::OcTree * octree)
{    
	//-----------------------------------------------------------------
	// *** F:DN variables
	//-----------------------------------------------------------------
    
    static std::random_device random_seed;
    static std::mt19937 rd_mt(random_seed()); //a pseudo-random number generator
    static std::uniform_real_distribution<> x_dist(x__low_bound__global, x__high_bound__global); 
	static std::uniform_real_distribution<> y_dist(y__low_bound__global, y__high_bound__global); 
	static std::uniform_real_distribution<> z_dist(z__low_bound__global, z__high_bound__global); 

    //-----------------------------------------------------------------
    // *** F:DB Body
    //----------------------------------------------------------------- 
	// Add random nodes
    std::vector<graph::node_id> nodes_added;
	while (nodes_added.size() < nodes_to_add_to_roadmap__global) {
		double x = x_dist(rd_mt), y = y_dist(rd_mt), z = z_dist(rd_mt);

		// Make sure we're not adding a node to an occupied part of the octomap
		if (!occupied(octree, x, y, z)) {			
			graph::node_id id = roadmap.add_node(x, y, z);
			nodes_added.push_back(id);
		}
	}

	// Connect the recently-added points to their neighbors in the roadmap
	for (const auto& n : nodes_added) {
		auto nearest_nodes = nodes_in_radius(roadmap, n, max_dist_to_connect_at__global, octree);
		for (const auto& n2 : nearest_nodes) {
            roadmap.connect(n, n2, dist(roadmap.get_node(n), roadmap.get_node(n2)));
		}
	}	
}


void create_response(package_delivery::get_trajectory::Response &res, smooth_trajectory& smooth_path)
{
    const double safe_radius = 1.0;

	// Sample trajectory
	mav_msgs::EigenTrajectoryPoint::Vector states;
	//double sampling_interval__global;
	//ros::param::get("/motion_planner/sampling_interval__global", sampling_interval__global);
	mav_trajectory_generation::sampleWholeTrajectory(smooth_path, sampling_interval__global, &states);

    // Get starting position
    graph::node start = {states[0].position_W.x(), states[0].position_W.y(), states[0].position_W.z()};

	// Convert sampled trajectory points to MultiDOFJointTrajectory response
	res.multiDOFtrajectory.joint_names.push_back("base");
    res.unknown = -1;

    int state_index = 0;
	for (const auto& s : states) {
		trajectory_msgs::MultiDOFJointTrajectoryPoint point;

		geometry_msgs::Transform pos;
        graph::node current;
		pos.translation.x = current.x = s.position_W.x();
		pos.translation.y = current.y = s.position_W.y();
		pos.translation.z = current.z = s.position_W.z();

		geometry_msgs::Twist vel;
		vel.linear.x = s.velocity_W.x();
		vel.linear.y = s.velocity_W.y();
		vel.linear.z = s.velocity_W.z();

		geometry_msgs::Twist acc;
		acc.linear.x = s.acceleration_W.x();
		acc.linear.y = s.acceleration_W.y();
		acc.linear.z = s.acceleration_W.z();

		ros::Duration dur(float(s.time_from_start_ns) / 1e9);

		point.transforms.push_back(pos);
		point.velocities.push_back(vel);
		point.accelerations.push_back(acc);
		point.time_from_start = dur;

        if (res.unknown != -1 &&
                !known(octree, current.x, current.y, current.z)
                && dist(start, current) > safe_radius) {
            ROS_WARN("Trajectory enters unknown space.");
            res.unknown = state_index;
        }

		res.multiDOFtrajectory.points.push_back(point);

        state_index++;
	}
}


smooth_trajectory smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree)
{
    // Variables for visualization for debugging purposes
	double distance = 0.5; 
	std::string frame_id = "world";

	// Setup optimizer
	mav_trajectory_generation::Vertex::Vector vertices;
	const int dimension = 3;
	const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
	
	// Convert roadmap path to optimizer's path format
	mav_trajectory_generation::Vertex start_v(dimension), end_v(dimension);
	start_v.makeStartOrEnd(Eigen::Vector3d(piecewise_path.front().x, piecewise_path.front().y, piecewise_path.front().z), derivative_to_optimize);
	end_v.makeStartOrEnd(Eigen::Vector3d(piecewise_path.back().x, piecewise_path.back().y, piecewise_path.back().z), derivative_to_optimize);

	vertices.push_back(start_v);
	for (auto it = piecewise_path.begin()+1; it+1 != piecewise_path.end(); ++it) {
		mav_trajectory_generation::Vertex v(dimension);
		v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(it->x, it->y, it->z));
		vertices.push_back(v);
	}
	vertices.push_back(end_v);

	// Parameters used to calculate how quickly the drone can move between vertices
	const double magic_fabian_constant = 6.5; // A tuning parameter.

	//double v_max__global, a_max__global;
	//ros::param::get("/motion_planner/v_max__global", v_max__global);
	//ros::param::get("/motion_planner/a_max__global", a_max__global);

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);

	// Optimize until no collisions are present
	bool col;
	do {
		ROS_INFO("Checking for collisions...");
		col = false;

		// Estimate the time the drone should take flying between each node
		auto segment_times = estimateSegmentTimes(vertices, v_max__global, a_max__global, magic_fabian_constant);
	
		// Optimize and create a smooth path from the vertices
		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();

		// Return all the smooth segments in the path
		// (Each segment goes from one of the original nodes to the next one in the path)
		mav_trajectory_generation::Segment::Vector segments;
		opt.getSegments(&segments);

		// Loop through the vector of segments looking for collisions
		for (int i = 0; !col && i < segments.size(); ++i) {
			const double time_step = 0.1;
			double segment_len = segments[i].getTime();

			auto segment_start = *(piecewise_path.begin() + i);
			auto segment_end = *(piecewise_path.begin() + i + 1);

			// Step through each individual segment, at increments of "time_step" seconds, looking for a collision
			for (double t = 0; t < segment_len - time_step; t += time_step) {
				auto pos1 = segments[i].evaluate(t);
				auto pos2 = segments[i].evaluate(t + time_step);

				graph::node n1 = {pos1.x(), pos1.y(), pos1.z()};
				graph::node n2 = {pos2.x(), pos2.y(), pos2.z()};

				// Check for a collision between two near points on the segment
				
                
            //if (motion_planning_core_str != "lawn_mower") {
                if (out_of_bounds(n1) || out_of_bounds(n2) || collision(octree, n1, n2)) {
					
                    
                    // Add a new vertex in the middle of the segment we are currently on
					mav_trajectory_generation::Vertex middle(dimension);

					double middle_x = (segment_start.x + segment_end.x) / 2;
					double middle_y = (segment_start.y + segment_end.y) / 2;
					double middle_z = (segment_start.z + segment_end.z) / 2;

					middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(middle_x, middle_y, middle_z));

					vertices.insert(vertices.begin()+i+1, middle);

                    // Add a new node to the piecewise path where the vertex is
                    graph::node middle_node = {middle_x, middle_y, middle_z};
					piecewise_path.insert(piecewise_path.begin()+i+1, middle_node);

					col = true;

					break;
				}
            //}
			}
		}
	} while (col);

	// Return the collision-free smooth trajectory
	mav_trajectory_generation::Trajectory traj;
	opt.getTrajectory(&traj);

	ROS_INFO("Smoothened path!");

	// Visualize path for debugging purposes
	mav_trajectory_generation::drawMavTrajectory(traj, distance, frame_id, &smooth_traj_markers);
	mav_trajectory_generation::drawVertices(vertices, frame_id, &piecewise_traj_markers);

	return traj;
}


void publish_graph(graph& g)
{
	pcl_ptr->clear();
    graph_conn_list.points.clear();

	for (auto n_id : g.node_ids()) {
		graph::node n = g.get_node(n_id);
		pcl_ptr->points.push_back (pcl::PointXYZ(n.x, n.y, n.z));

        // Publish edges between nodes
        for (const auto& e : g.adjacent_edges(n_id)) {
            geometry_msgs::Point p1, p2;

            p1.x = g.get_node(e.n1).x;
            p1.y = g.get_node(e.n1).y;
            p1.z = g.get_node(e.n1).z;

            p2.x = g.get_node(e.n2).x;
            p2.y = g.get_node(e.n2).y;
            p2.z = g.get_node(e.n2).z;

            graph_conn_list.points.push_back(p1);
            graph_conn_list.points.push_back(p2);
        }
	}

    if (DEBUG__global) {
        // Visualize graph immediately (a temporary debugging technique)
        graph_conn_pub.publish(graph_conn_list);
        ros::spinOnce();
    }
}


void postprocess(piecewise_trajectory& path)
{
    // We use a greedy approach to shorten the path here.
    // We connect non-adjacent nodes in the path that do not have collisions.
    
    for (auto it = path.begin(); it != path.end()-1; ) {
        bool shortened = false;
        for (auto it2 = path.end()-1; it2 != it+1 && !shortened; --it2) {
            if (!collision(octree, *it, *it2)) {
                it = path.erase(it+1, it2);
                shortened = true;
            }
        }

        if (!shortened)
            ++it;
    }
}

piecewise_trajectory lawn_mower(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
	//----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    piecewise_trajectory result;
	graph::node_id start_id, goal_id;
	auto generate_shortest_path = keep_roadmap_intact_plan; // TODO: parameter
	// auto generate_shortest_path = astar_plan;
	//int max_roadmap_size__global;

    //ros::param::get("motion_planner/max_roadmap_size__global", max_roadmap_size__global);
    
    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 
    graph roadmap = create_lawnMower_path(start, width, length, n_pts_per_dir, octree, start_id, goal_id);

    if (roadmap.size() == 0) {
    	ROS_ERROR("PRM could not be initialized.");
    	return result;
    }

    publish_graph(roadmap); // A debugging function used to publish the roadmap generated, so it can be viewed in rviz

    	
    // Search for a path to the goal in the PRM
    result = generate_shortest_path(roadmap);
/*
    // Grow PRM and re-run path-planner until we find a path
    while(result.empty()) {
        grow_PRM(roadmap, octree);

        ROS_INFO("Roadmap size: %d", roadmap.size());

        publish_graph(roadmap);

      	if (roadmap.size() > max_roadmap_size__global) {
            ROS_ERROR("Path not found!");
            return result;
      	}

		result = generate_shortest_path(roadmap, start_id, goal_id);
    }
    */
    return result;
}

piecewise_trajectory PRM(geometry_msgs::Point start, geometry_msgs::Point goal,int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
	//----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    piecewise_trajectory result;
	graph::node_id start_id, goal_id;
	auto generate_shortest_path = dijkstra_plan; // TODO: parameter
	// auto generate_shortest_path = astar_plan;
	//int max_roadmap_size__global;

    //ros::param::get("motion_planner/max_roadmap_size__global", max_roadmap_size__global);
    
    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 
    graph roadmap = create_PRM(start, goal, octree, start_id, goal_id);

    if (roadmap.size() == 0) {
    	ROS_ERROR("PRM could not be initialized.");
    	return result;
    }

    publish_graph(roadmap); // A debugging function used to publish the roadmap generated, so it can be viewed in rviz

	// Search for a path to the goal in the PRM
    result = generate_shortest_path(roadmap, start_id, goal_id);

    // Grow PRM and re-run path-planner until we find a path
    while(result.empty()) {
      	if (roadmap.size() > max_roadmap_size__global) {
            ROS_ERROR("Path not found!");
            return result;
      	}

        extend_PRM(roadmap, octree);

        ROS_INFO("Roadmap size: %u", roadmap.size());

        publish_graph(roadmap);

		result = generate_shortest_path(roadmap, start_id, goal_id);
    }

    return result;
}

graph create_RRT(geometry_msgs::Point start, graph::node_id &start_id)
{
	graph rrt;


	start_id = -1;
	graph::node s = {start.x, start.y, start.z, start_id};
    s.parent = graph::invalid_id();

    rrt.add_node(s);


	return rrt;
}

graph::node_id closest_node_to_coordinate(graph& g, double x, double y, double z)
{
    auto node_ids = g.node_ids();

    graph::node goal = {x, y, z};

    return *min_element(node_ids.begin(), node_ids.end(), [&g, &goal] (graph::node_id n1, graph::node_id n2) { return dist(g.get_node(n1), goal) < dist(g.get_node(n2), goal); });
}




graph::node_id extend_RRT(graph& rrt, geometry_msgs::Point goal, bool& reached_goal)
{
    graph::node_id result;
    reached_goal = false;

    static std::random_device random_seed;
    static std::mt19937 rd_mt(random_seed()); //a pseudo-random number generator
    static std::uniform_real_distribution<> x_dist(x__low_bound__global, x__high_bound__global); 
	static std::uniform_real_distribution<> y_dist(y__low_bound__global, y__high_bound__global); 
	static std::uniform_real_distribution<> z_dist(z__low_bound__global, z__high_bound__global); 
    static std::uniform_int_distribution<> bias_dist(0, 100);

    // Get random coordinate, q_random
    double x, y, z;
    bool towards_goal;

    if (bias_dist(rd_mt) <= rrt_bias__global) {
        x = goal.x, y = goal.y, z = goal.z;
        towards_goal = true;
    } else {
        x = x_dist(rd_mt), y = y_dist(rd_mt), z = z_dist(rd_mt);
        towards_goal = false;
    }
    
    // Find closest node, q_close
    graph::node_id q_close_id = closest_node_to_coordinate(rrt, x, y, z);

    // Get q_step
    graph::node& q_close = rrt.get_node(q_close_id);
    double dx = x - q_close.x;
    double dy = y - q_close.y;
    double dz = z - q_close.z;

    double d_len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (d_len > rrt_step_size__global) { 
        dx = dx * rrt_step_size__global / d_len;
        dy = dy * rrt_step_size__global / d_len;
        dz = dz * rrt_step_size__global / d_len;
    }

    graph::node q_step = {q_close.x + dx, q_close.y + dy, q_close.z + dz};

    // If no collision, append q_step to q_close
    if (!collision(octree, q_close, q_step)) {
        graph::node_id q_step_id = rrt.add_node(q_step.x, q_step.y, q_step.z);

        // The cost of an edge is not important for RRT, so we leave it 0
        rrt.connect(q_close_id, q_step_id, 0);
        rrt.get_node(q_step_id).parent = q_close_id;

        // Check whether we have reached the goal
        if (towards_goal && d_len <= rrt_step_size__global)
            reached_goal = true;

        result = q_step_id;
    } else {
        result = graph::invalid_id();
    }

    // Return result (either q_step or an invalid id)
    return result;
}

piecewise_trajectory build_reverse_path(graph g, graph::node_id goal)
{
    piecewise_trajectory result;

    for (graph::node_id n = goal; n != graph::invalid_id(); n = g.get_node(n).parent) {
        result.push_back(g.get_node(n));
    }

	std::reverse(result.begin(), result.end());

    return result;
}

piecewise_trajectory RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    piecewise_trajectory result;
    graph::node_id start_id, goal_id;

    // Initialize the RRT with the starting location
    graph rrt = create_RRT(start, start_id);

    // Keep extending the RRT until we reach the goal
    int iterations = 0;
    int fails = 0;
    for (bool finished = false; !finished;
            goal_id = extend_RRT(rrt, goal, finished)) {

        publish_graph(rrt);

        if (iterations >= 1000000000) {
            publish_graph(rrt);
            ROS_INFO("We're done ese");
            return result;
        }
        iterations++;

        if (goal_id == graph::invalid_id())
            fails++;
    }

    publish_graph(rrt);

    ROS_INFO("Fail rate: %f%%", double(fails*100) / iterations);

    ROS_INFO("RRT (of size %u) reached goal! Building path now", rrt.size());

    // Follow the parents on the tree backwards to the root
    result = build_reverse_path(rrt, goal_id);

    return result;
}

class OMPLMotionValidator : public ompl::base::MotionValidator
{
public:
    OMPLMotionValidator(const ompl::base::SpaceInformationPtr &si)
        : ompl::base::MotionValidator(si)
    {
    }

    bool checkMotion(const ompl::base::State *s1,
            const ompl::base::State *s2) const override
    {
        namespace ob = ompl::base;

        const auto *pos1 = s1->as<ob::RealVectorStateSpace::StateType>();
        const auto *pos2 = s2->as<ob::RealVectorStateSpace::StateType>();

        double x1 = pos1->values[0], x2 = pos2->values[0];
        double y1 = pos1->values[1], y2 = pos2->values[1];
        double z1 = pos1->values[2], z2 = pos2->values[2];

        return !collision(octree, {x1,y1,z1}, {x2,y2,z2});
    }

    bool checkMotion(const ompl::base::State *s1,
            const ompl::base::State *s2,
            std::pair<ompl::base::State*, double>& lastValid) const override
    {
        namespace ob = ompl::base;

        const auto *pos1 = s1->as<ob::RealVectorStateSpace::StateType>();
        const auto *pos2 = s2->as<ob::RealVectorStateSpace::StateType>();

        double x1 = pos1->values[0], x2 = pos2->values[0];
        double y1 = pos1->values[1], y2 = pos2->values[1];
        double z1 = pos1->values[2], z2 = pos2->values[2];

        graph::node end;
        bool valid = !collision(octree, {x1,y1,z1}, {x2,y2,z2}, &end);

        if (!valid) {
            auto *end_pos = lastValid.first->as<ob::RealVectorStateSpace::StateType>();
            end_pos->values[0] = end.x;
            end_pos->values[1] = end.y;
            end_pos->values[2] = end.z;

            double dx = x2-x1, dy = y2-y1, dz = z2-z1;
            double end_dx = end.x-x1, end_dy = end.y-y1, end_dz = end.z-z1;

            if (dx != 0)
                lastValid.second = end_dx / dx;
            else if (dy != 0)
                lastValid.second = end_dy / dy;
            else if (dz != 0)
                lastValid.second = end_dz / dz;
            else
                lastValid.second = 0;
        }

        return valid;
    }

};

bool OMPLStateValidityChecker(const ompl::base::State * state)
{
    namespace ob = ompl::base;

    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

    double x = pos->values[0];
    double y = pos->values[1];
    double z = pos->values[2];

    return !out_of_bounds({x,y,z}) && !occupied(octree, x, y, z);
}


template<class PlannerType>
piecewise_trajectory OMPL_plan(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
#ifndef INFLATE
    namespace ob = ompl::base;
    namespace og = ompl::geometric;

    piecewise_trajectory result;

    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // Set bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, std::min(x__low_bound__global, start.x));
    bounds.setHigh(0, std::max(x__high_bound__global, start.x));
    bounds.setLow(1, std::min(y__low_bound__global, start.y));
    bounds.setHigh(1, std::max(y__high_bound__global, start.y));
    bounds.setLow(2, std::min(z__low_bound__global, start.z));
    bounds.setHigh(2, std::max(z__high_bound__global, start.z));

    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    // Setup collision checker
    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    si->setStateValidityChecker(OMPLStateValidityChecker);
    si->setMotionValidator(std::make_shared<OMPLMotionValidator>(si));
    si->setup();

    // Set planner
    ob::PlannerPtr planner(new PlannerType(si));
    ss.setPlanner(planner);

    ob::ScopedState<> start_state(space);
    start_state[0] = start.x;
    start_state[1] = start.y;
    start_state[2] = start.z;

    ob::ScopedState<> goal_state(space);
    goal_state[0] = goal.x;
    goal_state[1] = goal.y;
    goal_state[2] = goal.z;

    ss.setStartAndGoalStates(start_state, goal_state);

    ss.setup();

    // Solve for path
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        ROS_INFO("Solution found!");
        ss.simplifySolution();

        for (auto state : ss.getSolutionPath().getStates()) {
            const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

            double x = pos->values[0];
            double y = pos->values[1];
            double z = pos->values[2];

            result.push_back({x, y, z});
        }
    }
    else
        ROS_ERROR("Path not found!");

    return result;
#else
    ROS_ERROR("OMPL-based planners cannot be compiled together with inflation!");
#endif
}


piecewise_trajectory OMPL_RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::RRT>(start, goal, width, length, n_pts_per_dir, octree);
}


piecewise_trajectory OMPL_RRTConnect(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::RRTConnect>(start, goal, width, length, n_pts_per_dir, octree);
}


piecewise_trajectory OMPL_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::PRM>(start, goal, width, length, n_pts_per_dir, octree);
}

