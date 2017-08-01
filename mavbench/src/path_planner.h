#ifndef AIRSIM_ROS_PATH_PLANNER_H
#define AIRSIM_ROS_PATH_PLANNER_H

#include <vector>
#include "graph.h"

std::vector<graph::node> dijkstra_plan(graph& g, graph::node_id start, graph::node_id goal);
std::vector<graph::node> astar_plan(graph& g, graph::node_id start, graph::node_id goal);

#endif
