#include <ros/ros.h>

// Standard headers
#include <cstdlib>
#include <cmath>
#include <random>
#include <vector>
#include <iostream>
#include <functional>
#include <limits>

// MAVBench headers
#include "common.h"
#include "graph.h"
#include "global_planner.h"
#include "package_delivery/get_trajectory.h"
#include "timer.h"
#include <profile_manager/profiling_data_srv.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/future_collision.h>

// Misc messages
#include <geometry_msgs/Point.h>

// Octomap specific headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

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

class MotionPlanner
{
friend class OMPLMotionValidator;

public:
    // Type-defs
    using piecewise_trajectory = std::vector<graph::node>;
    using smooth_trajectory = mav_trajectory_generation::Trajectory;

    MotionPlanner(octomap::OcTree * octree_) :
        octree(octree_)
    {
        motion_planning_initialize_params();

        // Topics and services
        get_trajectory_srv_server = nh.advertiseService("/get_trajectory_srv", &MotionPlanner::get_trajectory_fun, this);

        future_col_sub = nh.subscribe<mavbench_msgs::future_collision>("/col_coming", 1, &MotionPlanner::future_col_callback, this);

        smooth_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);
        piecewise_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
        // traj_pub = nh.advertise<mavbench_msgs::multiDOFtrajectory>("multidoftraj", 1);
    }

    void run() {}

    void log_data_before_shutting_down();

private:
    bool get_trajectory_fun(package_delivery::get_trajectory::Request &req, package_delivery::get_trajectory::Response &res);

    // ***F:DN Record which collision sequence we're on
    void future_col_callback(const mavbench_msgs::future_collision::ConstPtr& msg);

    // ***F:DN Create a grid-based lawn-mower-like path
    piecewise_trajectory lawn_mower(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);

    // ***F:DN Use the RRT sampling method from OMPL to find a piecewise path
    piecewise_trajectory OMPL_RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);

    // ***F:DN Use bi-directonal RRT from OMPL to find a piecewise path
    piecewise_trajectory OMPL_RRTConnect(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);

    // ***F:DN Use the PRM sampling method from OMPL to find a piecewise path
    piecewise_trajectory OMPL_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);

    // *** F:DN Checks whether a cell in the occupancy grid is occupied.
    bool occupied(octomap::OcTree * octree, double x, double y, double z);

    // *** F:DN Checks whether a cell in the occupancy grid is known.
    bool known(octomap::OcTree * octree, double x, double y, double z);

    // *** F:DN Checks whether there is a collision between two nodes in the occupancy grid.
    bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr = nullptr);

    // *** F:DN Checks whether a cell in the occupancy grid is occupied.
    bool out_of_bounds(const graph::node& pos);

    // *** F:DN Checks whether a cell in the occupancy grid is occupied.
    bool out_of_bounds_strict(const graph::node& pos);

    // *** F:DN Checks whether a cell in the occupancy grid is occupied.
    bool out_of_bounds_lax(const graph::node& pos);

    // *** F:DN Optimize and smoothen a piecewise path without causing any new collisions.
    smooth_trajectory smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree, Eigen::Vector3d initial_velocity, Eigen::Vector3d initial_acceleration);

    // ***F:DN Build the response to the service from the smooth_path
    void create_response(package_delivery::get_trajectory::Response &res, const smooth_trajectory& smooth_path);

    // ***F:DN Post-process piecewise path to optimize it.
    void postprocess(piecewise_trajectory& path);

    // *** F:DN initializing all the global variables 
    void motion_planning_initialize_params();

    // *** F:DN Check the validity of states for OMPL planners
    bool OMPLStateValidityChecker(const ompl::base::State * state);

    // *** F:DN A flexible wrapper for OMPL planners
    template<class PlannerType>
    piecewise_trajectory OMPL_plan(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree * octree);

private:
    ros::NodeHandle nh;
    ros::Publisher smooth_traj_vis_pub, piecewise_traj_vis_pub;
    ros::Subscriber future_col_sub;
    ros::ServiceServer get_trajectory_srv_server;
    // ros::Publisher traj_pub;

    octomap::OcTree * octree = nullptr;
    int future_col_seq_id = 0;

    geometry_msgs::Point g_start_pos;

    // Parameters
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
    long long g_planning_without_OM_PULL_time_acc = 0;
    int g_number_of_planning = 0 ;
    float g_planning_budget;
    float g_out_of_bounds_allowance = 5;

    // The following block of variables only exist for debugging purposes
    visualization_msgs::MarkerArray smooth_traj_markers;
    visualization_msgs::MarkerArray piecewise_traj_markers;
};


// A class to validate proposed motions for OMPL
class OMPLMotionValidator : public ompl::base::MotionValidator
{
public:
    OMPLMotionValidator(MotionPlanner * mp_, const ompl::base::SpaceInformationPtr &si)
        : ompl::base::MotionValidator(si), mp(mp_)
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

        return !mp->collision(mp->octree, {x1,y1,z1}, {x2,y2,z2});
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
        bool valid = !mp->collision(mp->octree, {x1,y1,z1}, {x2,y2,z2}, &end);

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

private:
    MotionPlanner * mp = nullptr;
};


template<class PlannerType>
MotionPlanner::piecewise_trajectory MotionPlanner::OMPL_plan(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree * octree)
{
    namespace ob = ompl::base;
    namespace og = ompl::geometric;

    piecewise_trajectory result;

    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // Set bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, std::min(x__low_bound__global, g_start_pos.x));
    bounds.setHigh(0, std::max(x__high_bound__global, g_start_pos.x));
    bounds.setLow(1, std::min(y__low_bound__global, g_start_pos.y));
    bounds.setHigh(1, std::max(y__high_bound__global, g_start_pos.y));
    bounds.setLow(2, std::min(z__low_bound__global, g_start_pos.z));
    bounds.setHigh(2, std::max(z__high_bound__global, g_start_pos.z));

    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    // Setup collision checker
    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    si->setStateValidityChecker([this] (const ompl::base::State * state) {
        return this->OMPLStateValidityChecker(state);
    });
    si->setMotionValidator(std::make_shared<OMPLMotionValidator>(this, si));
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
    ob::PlannerStatus solved = ss.solve(g_planning_budget);

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
}

