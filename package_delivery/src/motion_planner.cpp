#include "motion_planner.h"

//*** F:DN getting the smoothened trajectory
bool MotionPlanner::get_trajectory_fun(package_delivery::get_trajectory::Request &req, package_delivery::get_trajectory::Response &res)
{
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
	piecewise_trajectory piecewise_path;
	smooth_trajectory smooth_path;

    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 

    auto hook_end_t_2 = ros::Time::now(); 

    auto hook_start_t = ros::Time::now();

    g_start_time = ros::Time::now();
    g_start_pos = req.start;
    g_goal_pos = req.goal;
    
    piecewise_path = motion_planning_core(req.start, req.goal, req.width, req.length, req.n_pts_per_dir, octree);

    if (piecewise_path.empty()) {
        ROS_ERROR("Empty path returned");
        res.path_found = false;

        res.multiDOFtrajectory.future_collision_seq = future_col_seq_id;
        res.multiDOFtrajectory.trajectory_seq = trajectory_seq_id;
        trajectory_seq_id++;

        res.multiDOFtrajectory.append = false;
        res.multiDOFtrajectory.reverse = true;

        traj_pub.publish(res.multiDOFtrajectory);
        return true;
    }

    if (motion_planning_core_str != "lawn_mower") {
        postprocess(piecewise_path);
    }

    // Smoothen the path and build the multiDOFtrajectory response
    ROS_INFO("Smoothenning...");
    smooth_path = smoothen_the_shortest_path(piecewise_path, octree, 
                                    Eigen::Vector3d(req.twist.linear.x,
                                        req.twist.linear.y,
                                        req.twist.linear.z), 
                                    Eigen::Vector3d(req.acceleration.linear.x,
                                                    req.acceleration.linear.y,
                                                    req.acceleration.linear.z));

    if (smooth_path.empty()) {
        ROS_ERROR("Path could not be smoothened successfully");
        res.path_found = false;

        res.multiDOFtrajectory.future_collision_seq = future_col_seq_id;
        res.multiDOFtrajectory.trajectory_seq = trajectory_seq_id;
        trajectory_seq_id++;

        res.multiDOFtrajectory.append = false;
        res.multiDOFtrajectory.reverse = true;

        traj_pub.publish(res.multiDOFtrajectory);
        return true;
    }
	
    create_response(res, smooth_path);

    // Publish the trajectory (for debugging purposes)
    traj_pub.publish(res.multiDOFtrajectory);
    smooth_traj_vis_pub.publish(smooth_traj_markers);
    piecewise_traj_vis_pub.publish(piecewise_traj_markers);

    auto hook_end_t = ros::Time::now(); 
    g_planning_without_OM_PULL_time_acc += (((hook_end_t - hook_start_t).toSec())*1e9);
    g_number_of_planning++; 
    ROS_INFO("motion_planner: Planning took %f s", (hook_end_t - hook_start_t).toSec());

    res.path_found = true;
    return true;
}


void MotionPlanner::get_start_in_future(Drone& drone,
        geometry_msgs::Point& start, geometry_msgs::Twist& twist,
        geometry_msgs::Twist& acceleration)
{
    if (g_next_steps_msg.points.empty() || g_next_steps_msg.reverse) {
        auto pos = drone.position();
        start.x = pos.x; start.y = pos.y; start.z = pos.z; 
        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        return;
    }

    multiDOFpoint mdofp = trajectory_at_time(g_next_steps_msg, g_planning_budget);

    // Shift the drone's planned position at time "g_planning_budget" seconds
    // by its current position
    auto current_pos = drone.position();
    auto planned_pos = g_next_steps_msg.points[0];

    mdofp.x += current_pos.x - planned_pos.x;
    mdofp.y += current_pos.y - planned_pos.y;
    mdofp.z += current_pos.z - planned_pos.z;

    start.x = mdofp.x; start.y = mdofp.y; start.z = mdofp.z; 

    twist.linear.x = mdofp.vx;
    twist.linear.y = mdofp.vy;
    twist.linear.z = mdofp.vz;

    acceleration.linear.x = mdofp.ax;
    acceleration.linear.y = mdofp.ay;
    acceleration.linear.z = mdofp.az;
}


void MotionPlanner::future_col_callback(const mavbench_msgs::future_collision::ConstPtr& msg)
{
    ROS_INFO("motion_planner: New collision noticed");

    if (msg->future_collision_seq > future_col_seq_id)
        future_col_seq_id = msg->future_collision_seq;
    else
        return;

    // If neccessary, plan a new path for the drone
    if (g_next_steps_msg.future_collision_seq <= future_col_seq_id) {
        // "Call the get_trajectory_fun function right here, without waiting
        // for the package_delivery node to make a request
        package_delivery::get_trajectory::Request req;
        package_delivery::get_trajectory::Response res;

        get_start_in_future(*drone, req.start, req.twist, req.acceleration);
        req.goal = g_goal_pos;

        get_trajectory_fun(req, res);
    }
}

void MotionPlanner::next_steps_callback(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg)
{
    g_next_steps_msg = *msg;
}

void MotionPlanner::motion_planning_initialize_params()
{
    if(!ros::param::get("/planning_budget", g_planning_budget)){
      ROS_FATAL_STREAM("Could not start pkg delivery planning_budget not provided");
      return ;
    }
    
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
    
    ros::param::get("/motion_planner/motion_planning_core", motion_planning_core_str);
    if (motion_planning_core_str == "lawn_mower")
        motion_planning_core = [this] (geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree) {
            return this->lawn_mower(start, goal, width, length, n_pts_per_dir, octree);
        };
    else if (motion_planning_core_str == "OMPL-RRT")
        motion_planning_core = [this] (geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree) {
            return this->OMPL_RRT(start, goal, width, length, n_pts_per_dir, octree);
        };
    else if (motion_planning_core_str == "OMPL-RRTConnect")
        motion_planning_core = [this] (geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree) {
            return this->OMPL_RRTConnect(start, goal, width, length, n_pts_per_dir, octree);
        };
    else if (motion_planning_core_str == "OMPL-PRM")
        motion_planning_core = [this] (geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree) {
            return this->OMPL_PRM(start, goal, width, length, n_pts_per_dir, octree);
        };
    else {
        std::cout<<"This motion planning type is not defined"<<std::endl;
        exit(0);
    }
}

void MotionPlanner::log_data_before_shutting_down()
{
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

    profiling_data_srv_inst.request.key = "motion_planning_acc";
    profiling_data_srv_inst.request.value = (double)g_planning_without_OM_PULL_time_acc/1e9;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "motion_planning_kernel";
    profiling_data_srv_inst.request.value = ((double)g_planning_without_OM_PULL_time_acc/1e9)/g_number_of_planning;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
}

static double dist(const graph::node& n1, const graph::node& n2)
{
	return std::sqrt((n1.x-n2.x)*(n1.x-n2.x) + (n1.y-n2.y)*(n1.y-n2.y) + (n1.z-n2.z)*(n1.z-n2.z));
}


bool MotionPlanner::occupied(octomap::OcTree * octree, double x, double y, double z)
{
	const double OCC_THRESH = 0.5;

	octomap::OcTreeNode * otn = octree->search(x, y, z);

	return otn != nullptr && otn->getOccupancy() >= OCC_THRESH;
}


bool MotionPlanner::known(octomap::OcTree * octree, double x, double y, double z)
{
	return octree->search(x, y, z) != nullptr;
}

bool is_between(double x, double min, double max)
{
    return x >= min && x <= max;
}

bool MotionPlanner::out_of_bounds(const graph::node& pos)
{
    bool x_correct = is_between(pos.x, x__low_bound__global, x__high_bound__global);
    bool y_correct = is_between(pos.y, y__low_bound__global, y__high_bound__global);
    bool z_correct = is_between(pos.z, z__low_bound__global, z__high_bound__global);

    if (x_correct && y_correct && z_correct)
        return false;

    bool x_start_correct = is_between(g_start_pos.x, x__low_bound__global, x__high_bound__global);
    bool y_start_correct = is_between(g_start_pos.y, y__low_bound__global, y__high_bound__global);
    bool z_start_correct = is_between(g_start_pos.z, z__low_bound__global, z__high_bound__global);

    if (x_start_correct && y_start_correct && !z_start_correct) {
        if (is_between(pos.x, g_start_pos.x - g_out_of_bounds_allowance,
                    g_start_pos.x + g_out_of_bounds_allowance)
                && is_between(pos.y, g_start_pos.y - g_out_of_bounds_allowance,
                    g_start_pos.y + g_out_of_bounds_allowance)) {
            if (g_start_pos.z < z__low_bound__global && pos.z >= g_start_pos.z)
                return false;
            else if (g_start_pos.z > z__high_bound__global && pos.z <= g_start_pos.z)
                return false;
        }
    }

    return true;
}


bool MotionPlanner::out_of_bounds_strict(const graph::node& pos)
{
    bool x_correct = is_between(pos.x, x__low_bound__global, x__high_bound__global);
    bool y_correct = is_between(pos.y, y__low_bound__global, y__high_bound__global);
    bool z_correct = is_between(pos.z, z__low_bound__global, z__high_bound__global);

    return !x_correct || !y_correct || !z_correct;
}


bool MotionPlanner::out_of_bounds_lax(const graph::node& pos)
{
    double x_low = std::min(g_start_pos.x, x__low_bound__global);
    double x_high = std::max(g_start_pos.x, x__high_bound__global);
    double y_low = std::min(g_start_pos.y, y__low_bound__global);
    double y_high = std::max(g_start_pos.y, y__high_bound__global);
    double z_low = std::min(g_start_pos.z, z__low_bound__global);
    double z_high = std::max(g_start_pos.z, z__high_bound__global);

    bool x_correct = is_between(pos.x, x_low, x_high);
    bool y_correct = is_between(pos.y, y_low, y_high);
    bool z_correct = is_between(pos.z, z_low, z_high);

    return !x_correct || !y_correct || !z_correct;
}


bool MotionPlanner::collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr)
{
    if (motion_planning_core_str == "lawn_mower")
        return false;
    
    RESET_TIMER();

    // First, check if anything goes too close to the ground
    if (n1.z <= drone_height__global || n2.z <= drone_height__global)
        return true;

    // Next, check if it goes out-of-bounds
    bool z_out_of_bounds = !is_between(n1.z, z__low_bound__global, z__high_bound__global)
        || !is_between(n2.z, z__low_bound__global, z__high_bound__global);
    if (z_out_of_bounds) {
        if (!is_between(n1.x, n2.x - g_out_of_bounds_allowance, n2.x + g_out_of_bounds_allowance)
                || !is_between(n1.y, n2.y - g_out_of_bounds_allowance, n2.y + g_out_of_bounds_allowance)) {
            if (end_ptr != nullptr) {
                end_ptr->x = n1.x;
                end_ptr->y = n1.y;
                end_ptr->z = n1.z;
            }
            return true;
        }
    }

    // Create a bounding box representing the drone
    double height = drone_height__global; 
    double radius = drone_radius__global; 

    octomap::point3d min(n1.x-radius, n1.y-radius, n1.z-height/2);
    octomap::point3d max(n1.x+radius, n1.y+radius, n1.z+height/2);

    // Create a direction vector over which to check for collisions
	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;
	double dz = n2.z - n1.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    octomap::point3d direction(dx, dy, dz);

    // Make sure the direction vector isn't just (0,0,0)
    // Otherwise, we'll get a bunch of really annoying error messages
    // if (distance == 0) {
    //     if (occupied(octree, n1.x, n1.y, n1.z)) {
    //         if (end_ptr != nullptr) {
    //             end_ptr->x = n1.x;
    //             end_ptr->y = n1.y;
    //             end_ptr->z = n1.z;
    //         }
    //         return true;
    //     } else
    //         return false;
    // }

    // Finally, loop over the drone's bounding box to search for collisions
    octomap::point3d end;
    for (auto it = octree->begin_leafs_bbx(min, max),
            end_it = octree->end_leafs_bbx(); it != end_it; ++it)
    {
        octomap::point3d start (it.getCoordinate());
        
        // std::cout << distance << " (" << start.x() << " " << start.y() << " " << start.z() << ") (" << direction.x() << " " << direction.y() << " " << direction.z() << ")" << std::endl;

        if (octree->castRay(start, direction, end, true, distance)) {
            if (end_ptr != nullptr) {
                end_ptr->x = end.x();
                end_ptr->y = end.y();
                end_ptr->z = end.z();
            }

            return true;
        }
    }

	//LOG_ELAPSED(motion_planner);
	return false;
}


void MotionPlanner::create_response(package_delivery::get_trajectory::Response &res, const smooth_trajectory& smooth_path)
{
    const double safe_radius = 1.0;

	// Sample trajectory
	mav_msgs::EigenTrajectoryPoint::Vector states;
	// double sampling_interval__global;
	// ros::param::get("/motion_planner/sampling_interval__global", sampling_interval__global);
	mav_trajectory_generation::sampleWholeTrajectory(smooth_path, sampling_interval__global, &states);

    // Get starting position
    graph::node start = {states[0].position_W.x(), states[0].position_W.y(), states[0].position_W.z()};

	// Convert sampled trajectory points to MultiDOFJointTrajectory response
    res.unknown = -1;

    int state_index = 0;
	// for (const auto& s : states) {
    for (int i = 0; i < states.size() - 1; i++) {
        const auto& s = states[i];
        const auto& s_next = states[i+1];

		mavbench_msgs::multiDOFpoint point;

        graph::node current;
		point.x = current.x = s_next.position_W.x();
		point.y = current.y = s_next.position_W.y();
		point.z = current.z = s_next.position_W.z();

		point.vx = s.velocity_W.x();
		point.vy = s.velocity_W.y();
		point.vz = s.velocity_W.z();

		point.ax = s.acceleration_W.x();
		point.ay = s.acceleration_W.y();
		point.az = s.acceleration_W.z();

        point.yaw = yawFromVelocity(point.vx, point.vy);
        point.blocking_yaw = false;

	    point.duration = double(s_next.time_from_start_ns - s.time_from_start_ns) / 1e9;

        if (res.unknown != -1 &&
                !known(octree, current.x, current.y, current.z)
                && dist(start, current) > safe_radius) {
            ROS_WARN("Trajectory enters unknown space.");
            res.unknown = state_index;
        }

		res.multiDOFtrajectory.points.push_back(point);
        state_index++;
	}

    res.multiDOFtrajectory.append = false;
    res.multiDOFtrajectory.reverse = false;

    // Mark the trajectory with the correct sequence id's
    res.multiDOFtrajectory.trajectory_seq = trajectory_seq_id;
    trajectory_seq_id++;

    res.multiDOFtrajectory.future_collision_seq = future_col_seq_id;
}


MotionPlanner::smooth_trajectory MotionPlanner::smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree, Eigen::Vector3d initial_velocity, Eigen::Vector3d initial_acceleration)
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
	//start_v.makeStartOrEnd(Eigen::Vector3d(piecewise_path.front().x, piecewise_path.front().y, piecewise_path.front().z), derivative_to_optimize);
   	start_v.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, initial_velocity);
   	//start_v.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(3, 0, 0));
    start_v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(piecewise_path.front().x, piecewise_path.front().y, piecewise_path.front().z));
    
    end_v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(piecewise_path.back().x, piecewise_path.back().y, piecewise_path.back().z));
   	end_v.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
    ///Eigen::Vector3d(piecewise_path.front().x, piecewise_path.front().y, piecewise_path.front().z));
    //end_v.makeStartOrEnd(Eigen::Vector3d(piecewise_path.back().x, piecewise_path.back().y, piecewise_path.back().z), derivative_to_optimize);

	vertices.push_back(start_v);
	for (auto it = piecewise_path.begin()+1; it+1 != piecewise_path.end(); ++it) {
		mav_trajectory_generation::Vertex v(dimension);
		v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(it->x, it->y, it->z));
		vertices.push_back(v);
	}
	vertices.push_back(end_v);

	// Parameters used to calculate how quickly the drone can move between vertices
	const double magic_fabian_constant = 6.5; // A tuning parameter.

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);

	// Optimize until no collisions are present
	bool col;
	do {
		col = false;

		// Estimate the time the drone should take flying between each node
		auto segment_times = estimateSegmentTimes(vertices, v_max__global, a_max__global, magic_fabian_constant);

        for (auto& el : segment_times)
            el *= 0.5;

		// Optimize and create a smooth path from the vertices
		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();

		// Return all the smooth segments in the path
		// (Each segment goes from one of the original nodes to the next one in the path)
		mav_trajectory_generation::Segment::Vector segments;
		opt.getSegments(&segments);

		// Loop through the vector of segments looking for collisions
		for (int i = 0; !col && i < segments.size(); ++i) {
            // ROS_INFO("Looping through segments...");
			const double time_step = 0.1;
			double segment_len = segments[i].getTime();

			auto segment_start = *(piecewise_path.begin() + i);
			auto segment_end = *(piecewise_path.begin() + i + 1);

			// Step through each individual segment, at increments of "time_step" seconds, looking for a collision
			for (double t = 0; t < segment_len - time_step; t += time_step) {
                // ROS_INFO("Stepping through individual...");
				auto pos1 = segments[i].evaluate(t);
				auto pos2 = segments[i].evaluate(t + time_step);

				graph::node n1 = {pos1.x(), pos1.y(), pos1.z()};
				graph::node n2 = {pos2.x(), pos2.y(), pos2.z()};

				// Check for a collision between two near points on the segment

                //if (motion_planning_core_str != "lawn_mower") {
                if (out_of_bounds_lax(n1) || out_of_bounds_lax(n2) || collision(octree, n1, n2)) {

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
	} while (col &&
            ros::Time::now() < g_start_time+ros::Duration(g_planning_budget));

    if (col)
        return smooth_trajectory();

	// Return the collision-free smooth trajectory
	mav_trajectory_generation::Trajectory traj;
	opt.getTrajectory(&traj);

	//ROS_INFO("Smoothened path!");
	// Visualize path for debugging purposes
	mav_trajectory_generation::drawMavTrajectory(traj, distance, frame_id, &smooth_traj_markers);
	mav_trajectory_generation::drawVertices(vertices, frame_id, &piecewise_traj_markers);

	return traj;
}


void MotionPlanner::postprocess(piecewise_trajectory& path)
{
    // We use a greedy approach to shorten the path here.
    // We connect non-adjacent nodes in the path that do not have collisions.
    
    for (auto it = path.begin(); it != path.end()-1; ) {
        // if (out_of_bounds_strict(*it)) {
        //     ++it;
        //     continue;
        // }

        bool shortened = false;
        for (auto it2 = path.end()-1; it2 != it+1 && !shortened; --it2) {
            if (!collision(octree, *it, *it2)
                && out_of_bounds_strict(*it) == out_of_bounds_strict(*it2)) {
                it = path.erase(it+1, it2);
                shortened = true;
            }
        }

        if (!shortened)
            ++it;
    }
}


static graph create_lawnMower_path(geometry_msgs::Point start, int width, int length, int n_pts_per_dir, octomap::OcTree *octree, graph::node_id &start_id, graph::node_id &goal_id)

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
    /*   
	if (occupied(octree, start.x, start.y, start.z)) {
		ROS_ERROR("Start is already occupied!");
		success = false;
	}
    */ 
    return roadmap;
}


MotionPlanner::piecewise_trajectory MotionPlanner::lawn_mower(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
	//----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    piecewise_trajectory result;
	graph::node_id start_id, goal_id;
	auto generate_shortest_path = keep_roadmap_intact_plan; // TODO: parameter

    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 
    graph roadmap = create_lawnMower_path(start, width, length, n_pts_per_dir, octree, start_id, goal_id);

    if (roadmap.size() == 0) {
    	ROS_ERROR("PRM could not be initialized.");
    	return result;
    }

    // publish_graph(roadmap); // A debugging function used to publish the roadmap generated, so it can be viewed in rviz

    // Search for a path to the goal in the PRM
    result = generate_shortest_path(roadmap);

    return result;
}


bool MotionPlanner::OMPLStateValidityChecker(const ompl::base::State * state)
{
    namespace ob = ompl::base;

    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

    double x = pos->values[0];
    double y = pos->values[1];
    double z = pos->values[2];

    return !out_of_bounds({x,y,z}) && !occupied(octree, x, y, z);
}


MotionPlanner::piecewise_trajectory MotionPlanner::OMPL_RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::RRT>(start, goal, octree);
}


MotionPlanner::piecewise_trajectory MotionPlanner::OMPL_RRTConnect(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::RRTConnect>(start, goal, octree);
}


MotionPlanner::piecewise_trajectory MotionPlanner::OMPL_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
    return OMPL_plan<ompl::geometric::PRM>(start, goal, octree);
}

