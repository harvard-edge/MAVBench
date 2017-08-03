#include <ros/ros.h>

// Standard headers
#include <cstdlib>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include <iostream>
#include <thread>
// My headers
#include "common.h"
#include "Drone.h"
#include "graph.h"
#include "global_planner.h"
#include "package_delivery/get_trajectory.h"

// Misc messages
#include <geometry_msgs/Point.h>

// Octomap specific includes
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

// TF specific includes
#include <tf/transform_datatypes.h>

// Pointcloud headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Trajectory smoothening includes
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>

// Type-defs
using piecewise_trajectory = std::vector<graph::node>;
using smooth_trajectory = mav_trajectory_generation::Trajectory;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

//*** F:DN global variables
octomap::OcTree * octree = nullptr;
trajectory_msgs::MultiDOFJointTrajectory traj_topic;

// The following block of global variables only exist for debugging purposes
visualization_msgs::MarkerArray smooth_traj_markers;
visualization_msgs::MarkerArray piecewise_traj_markers;
octomap_msgs::Octomap omp;
PointCloud::Ptr pcl_ptr{new pcl::PointCloud<pcl::PointXYZ>};
visualization_msgs::Marker graph_conn_list;
ros::Publisher graph_conn_pub;

// *** F:DN calculating the distance between two nodes in the graph.
double dist(const graph::node& n1, const graph::node& n2);


// *** F:DN Checks whether a cell in the occupancy grid is occupied.
bool occupied(octomap::OcTree * octree, double x, double y, double z);


// *** F:DN Checks whether a cell in the occupancy grid is known.
bool known(octomap::OcTree * octree, double x, double y, double z);


// *** F:DN Checks whether there is a collision between two nodes in the occupancy grid.
bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2);


// *** F:DN find all neighbours within "max_dist" meters of node
std::vector<graph::node_id> nodes_in_radius(/*const*/ graph& g, graph::node_id n, double max_dist, octomap::OcTree * octree);


// *** F:DN Generatea octomap. Returns nullptr on error.
void generate_octomap(const octomap_msgs::Octomap& msg);


// *** F:DN Initializes the PRM.
graph create_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree *octree, graph::node_id &start_id, graph::node_id &goal_id);


// *** F:DN Increases the density of the PRM.
void grow_PRM(graph &roadmap, octomap::OcTree * octree);


// ***F:DN Use the PRM sampling method to find a piecewise path
piecewise_trajectory PRM(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree * octree);


// ***F:DN Use the RRT sampling method to find a piecewise path
piecewise_trajectory RRT(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree * octree);


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
	//----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
	piecewise_trajectory piecewise_path;
	smooth_trajectory smooth_path;
    auto motion_planning_core = RRT;


    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 

    if (octree == nullptr) {
    	ROS_ERROR("Octomap is not available.");
    	return false;
    }

    piecewise_path = motion_planning_core(req.start, req.goal, octree);

    if (piecewise_path.size() == 0) {
        ROS_ERROR("Empty path returned");
        return false;
    }

    ROS_INFO("Path size: %d. Now post-processing...", piecewise_path.size());

    postprocess(piecewise_path);

    ROS_INFO("Path size: %d. Now smoothening...", piecewise_path.size());

    // Smoothen the path and build the multiDOFtrajectory response
    smooth_path = smoothen_the_shortest_path(piecewise_path, octree);
	
    create_response(res, smooth_path);

    // Publish the trajectory (for debugging purposes)
    traj_topic = res.multiDOFtrajectory;

	return true;
}


int main(int argc, char ** argv)
{
    //----------------------------------------------------------------- 
    // *** F:DN variables	
    //----------------------------------------------------------------- 
    graph::node_id start_id,  goal_id;
    graph roadmap;
    std::vector<graph::node> piecewise_path;
	octomap::OcTree * octree;
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;

    ros::Subscriber octomap_sub = nh.subscribe("octomap_full", 1, generate_octomap);
    ros::ServiceServer service = nh.advertiseService("get_trajectory_srv", get_trajectory_fun);
    ros::Publisher smooth_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);
    ros::Publisher piecewise_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("multidoftraj", 1);
	ros::Publisher octo_pub = nh.advertise<octomap_msgs::Octomap>("omap", 1);
    ros::Publisher pcl_pub = nh.advertise<PointCloud> ("graph", 1);
    graph_conn_pub = nh.advertise<visualization_msgs::Marker>("graph_conns", 100);
	
	pcl_ptr->header.frame_id = graph_conn_list.header.frame_id = "fcu";
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


	ros::Rate pub_rate(5);
	while (ros::ok())
	{
        smooth_traj_vis_pub.publish(smooth_traj_markers);
		piecewise_traj_vis_pub.publish(piecewise_traj_markers);
		octo_pub.publish(omp);
		pcl_pub.publish(pcl_ptr);
        graph_conn_pub.publish(graph_conn_list);
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


bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2)
{
	const double pi = 3.14159265359;

	// The drone is modeled as a cylinder.
	// Angles are in radians and lengths are in meters.

    static double height = [] () {
        double h;
        ros::param::get("/motion_planner/drone_height", h);
        return h;
    } ();

    static double radius = [] () {
        double r;
        ros::param::get("/motion_planner/drone_radius", r);
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

				if (octree->castRay(start, direction, end, true, distance))
					return true;
			}
		}
	}

	return false;
}

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


void generate_octomap(const octomap_msgs::Octomap& msg)
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
		ROS_ERROR("Start is already occupied!");
		success = false;
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


void grow_PRM(graph &roadmap, octomap::OcTree * octree)
{    
	//-----------------------------------------------------------------
	// *** F:DN parameters 
	//-----------------------------------------------------------------
	int nodes_to_add_to_roadmap;
	double max_dist_to_connect_at;
    double x_dist_to_sample_from__low_bound, x_dist_to_sample_from__high_bound;
    double y_dist_to_sample_from__low_bound, y_dist_to_sample_from__high_bound;
    double z_dist_to_sample_from__low_bound, z_dist_to_sample_from__high_bound;

	// Move these into main() to avoid continously incurring communication overhead with ROS parameter server
	ros::param::get("/motion_planner/nodes_to_add_to_roadmap", nodes_to_add_to_roadmap);
	ros::param::get("/motion_planner/max_dist_to_connect_at", max_dist_to_connect_at);

	//-----------------------------------------------------------------
	// *** F:DN variables
	//-----------------------------------------------------------------
    ros::param::get("/motion_planner/x_dist_to_sample_from__low_bound", x_dist_to_sample_from__low_bound);
	ros::param::get("/motion_planner/x_dist_to_sample_from__high_bound", x_dist_to_sample_from__high_bound);
	ros::param::get("/motion_planner/y_dist_to_sample_from__low_bound", y_dist_to_sample_from__low_bound);
	ros::param::get("/motion_planner/y_dist_to_sample_from__high_bound", y_dist_to_sample_from__high_bound);
	ros::param::get("/motion_planner/z_dist_to_sample_from__low_bound", z_dist_to_sample_from__low_bound);
	ros::param::get("/motion_planner/z_dist_to_sample_from__high_bound", z_dist_to_sample_from__high_bound);
    static std::mt19937 rd_mt(350); //a pseudo-random number generator
    static std::uniform_real_distribution<> x_dist(x_dist_to_sample_from__low_bound, x_dist_to_sample_from__high_bound); 
	static std::uniform_real_distribution<> y_dist(y_dist_to_sample_from__low_bound, y_dist_to_sample_from__high_bound); 
	static std::uniform_real_distribution<> z_dist(z_dist_to_sample_from__low_bound, z_dist_to_sample_from__high_bound); 

    //-----------------------------------------------------------------
    // *** F:DB Body
    //----------------------------------------------------------------- 
	// Add random nodes
    std::vector<graph::node_id> nodes_added;
	while (nodes_added.size() < nodes_to_add_to_roadmap) {
		double x = x_dist(rd_mt), y = y_dist(rd_mt), z = z_dist(rd_mt);

		// Make sure we're not adding a node to an occupied part of the octomap
		if (!occupied(octree, x, y, z)) {			
			graph::node_id id = roadmap.add_node(x, y, z);
			nodes_added.push_back(id);
		}
	}

	// Connect the recently-added points to their neighbors in the roadmap
	for (const auto& n : nodes_added) {
		auto nearest_nodes = nodes_in_radius(roadmap, n, max_dist_to_connect_at, octree);

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
	double sampling_interval;
	ros::param::get("/motion_planner/sampling_interval", sampling_interval);
	mav_trajectory_generation::sampleWholeTrajectory(smooth_path, sampling_interval, &states);

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
	std::string frame_id = "fcu";

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
	double v_max, a_max;
	const double magic_fabian_constant = 6.5; // A tuning parameter.

	ros::param::get("/motion_planner/v_max", v_max);
	ros::param::get("/motion_planner/a_max", a_max);

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);

	// Optimize until no collisions are present
	bool col;
	do {
		ROS_INFO("Checking for collisions...");
		col = false;

		// Estimate the time the drone should take flying between each node
		auto segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);
	
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
				if (collision(octree, n1, n2)) {
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
			}
		}
	} while (col);

	// Return the collision-free smooth trajectory
	mav_trajectory_generation::Trajectory trajectory;
	opt.getTrajectory(&trajectory);

	ROS_INFO("Smoothened path!");

	// Visualize path for debugging purposes
	mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &smooth_traj_markers);
	mav_trajectory_generation::drawVertices(vertices, frame_id, &piecewise_traj_markers);

	return trajectory;
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
}


void postprocess(piecewise_trajectory& path)
{
    // We use the greedy approach to shorten the path here.
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

piecewise_trajectory PRM(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree * octree)
{
	//----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    piecewise_trajectory result;
	graph::node_id start_id, goal_id;
	auto generate_shortest_path = dijkstra_plan; // TODO: parameter
	// auto generate_shortest_path = astar_plan;
	int max_roadmap_size;

    ros::param::get("motion_planner/max_roadmap_size", max_roadmap_size);
    
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
        grow_PRM(roadmap, octree);

        ROS_INFO("Roadmap size: %d", roadmap.size());

        publish_graph(roadmap);

      	if (roadmap.size() > max_roadmap_size) {
            ROS_ERROR("Path not found!");
            return result;
      	}

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

    static double step_size;
    static int bias;

    static double x_dist_to_sample_from__low_bound, x_dist_to_sample_from__high_bound;
    static double y_dist_to_sample_from__low_bound, y_dist_to_sample_from__high_bound;
    static double z_dist_to_sample_from__low_bound, z_dist_to_sample_from__high_bound;
    static bool initialized = false;

    if (!initialized) {
        ros::param::get("/motion_planner/rrt_step_size", step_size);
        ros::param::get("/motion_planner/rrt_bias", bias);

        ros::param::get("/motion_planner/x_dist_to_sample_from__low_bound", x_dist_to_sample_from__low_bound);
        ros::param::get("/motion_planner/x_dist_to_sample_from__high_bound", x_dist_to_sample_from__high_bound);
        ros::param::get("/motion_planner/y_dist_to_sample_from__low_bound", y_dist_to_sample_from__low_bound);
        ros::param::get("/motion_planner/y_dist_to_sample_from__high_bound", y_dist_to_sample_from__high_bound);
        ros::param::get("/motion_planner/z_dist_to_sample_from__low_bound", z_dist_to_sample_from__low_bound);
        ros::param::get("/motion_planner/z_dist_to_sample_from__high_bound", z_dist_to_sample_from__high_bound);

        initialized = true;
    }

    static std::mt19937 rd_mt(350); //a pseudo-random number generator
    static std::uniform_real_distribution<> x_dist(x_dist_to_sample_from__low_bound, x_dist_to_sample_from__high_bound); 
	static std::uniform_real_distribution<> y_dist(y_dist_to_sample_from__low_bound, y_dist_to_sample_from__high_bound); 
	static std::uniform_real_distribution<> z_dist(z_dist_to_sample_from__low_bound, z_dist_to_sample_from__high_bound); 
    static std::uniform_int_distribution<> bias_dist(0, 100);

    // Get random coordinate, q_random
    double x, y, z;
    bool towards_goal;

    if (bias_dist(rd_mt) <= bias) {
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
    if (d_len > step_size) { 
        dx = dx * step_size / d_len;
        dy = dy * step_size / d_len;
        dz = dz * step_size / d_len;
    }

    graph::node q_step = {q_close.x + dx, q_close.y + dy, q_close.z + dz};

    // If no collision, append q_step to q_close
    if (!collision(octree, q_close, q_step)) {
        graph::node_id q_step_id = rrt.add_node(q_step.x, q_step.y, q_step.z);

        // The cost of an edge is not important for RRT, so we leave it 0
        rrt.connect(q_close_id, q_step_id, 0);
        rrt.get_node(q_step_id).parent = q_close_id;

        // Check whether we have reached the goal
        if (towards_goal && d_len <= step_size)
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

piecewise_trajectory RRT(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree * octree)
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

        // Visualize RRT (a temporary debugging technique)
        // publish_graph(rrt);
        // graph_conn_pub.publish(graph_conn_list);
        // ros::spinOnce();

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

    ROS_INFO("RRT (of size %d) reached goal! Building path now", rrt.size());

    // Follow the parents on the tree backwards to the root
    result = build_reverse_path(rrt, goal_id);

    return result;
}

