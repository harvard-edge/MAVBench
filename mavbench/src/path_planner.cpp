#include "path_planner.h"
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <vector>
#include <cmath>
#include <iostream>

std::vector<graph::node> dijkstra_plan(graph& g, graph::node_id start, graph::node_id goal)
{
	auto node_ids = g.node_ids();
	std::set<graph::node_id> unvisited(node_ids.begin(), node_ids.end());
	std::unordered_map<graph::node_id, double> cost;
	std::unordered_map<graph::node_id, graph::node_id> prev;

	for (auto n : node_ids) {
		cost[n] = std::numeric_limits<double>::infinity();
	}

	cost[start] = 0;
	unvisited.erase(start);

	graph::node_id current = start;

	while (current != goal && cost[current] != std::numeric_limits<double>::infinity()) {
		auto edges = g.adjacent_edges(current);

		for (const auto& e : edges) {
			graph::node_id connected_node = e.n1 != current ? e.n1 : e.n2;

			if (unvisited.find(connected_node) == unvisited.end())
				continue;

			double tentative_cost = cost[current] + e.cost;

			if (tentative_cost < cost[connected_node]) {
				cost[connected_node] = tentative_cost;
				prev[connected_node] = current;
			}
		}

		unvisited.erase(current);

		current = *std::min_element(unvisited.begin(), unvisited.end(),
			[&cost] (graph::node_id n1, graph::node_id n2) { return cost[n1] < cost[n2]; });
	}

	// Check for failure
	if (cost[current] == std::numeric_limits<double>::infinity())
		return std::vector<graph::node>();

	// Rebuild path
	std::vector<graph::node> result;

	current = goal;
	while(current != start) {
		result.push_back(g.get_node(current));
		current = prev[current];
	}
	result.push_back(g.get_node(start));
	std::reverse(result.begin(), result.end());

	return result;
}


std::vector<graph::node> astar_plan(graph& g, graph::node_id start, graph::node_id goal)
{
    // Define the heuristic function
    const auto h = [&g, goal](graph::node_id n_id) {
        auto n = g.get_node(n_id);
        auto gl = g.get_node(goal);

        return std::sqrt((n.x-gl.x)*(n.x-gl.x) + (n.y-gl.y)*(n.y-gl.y) + (n.z-gl.z)*(n.z-gl.z));
    };

    // Initialize all the required data structures
    auto node_ids = g.node_ids();
	std::set<graph::node_id> unvisited(node_ids.begin(), node_ids.end());
	std::unordered_map<graph::node_id, double> g_cost;
	std::unordered_map<graph::node_id, double> f_cost;
	std::unordered_map<graph::node_id, graph::node_id> prev;

	for (auto n : node_ids) {
		g_cost[n] = std::numeric_limits<double>::infinity();
		f_cost[n] = std::numeric_limits<double>::infinity();
	}

	g_cost[start] = 0;
    f_cost[start] = h(start);
	unvisited.erase(start);

	graph::node_id current = start;

	while (current != goal && g_cost[current] != std::numeric_limits<double>::infinity()) {
		auto edges = g.adjacent_edges(current);

		for (const auto& e : edges) {
			graph::node_id connected_node = e.n1 != current ? e.n1 : e.n2;

			if (unvisited.find(connected_node) == unvisited.end())
				continue;

			double tentative_g_cost = g_cost[current] + e.cost;

			if (tentative_g_cost < g_cost[connected_node]) {
				g_cost[connected_node] = tentative_g_cost;
                f_cost[connected_node] = tentative_g_cost + h(connected_node);
				prev[connected_node] = current;
			}
		}

		unvisited.erase(current);

		current = *std::min_element(unvisited.begin(), unvisited.end(),
			[&f_cost] (graph::node_id n1, graph::node_id n2) { return f_cost[n1] < f_cost[n2]; });
	}

	// Check for failure
	if (g_cost[current] == std::numeric_limits<double>::infinity())
		return std::vector<graph::node>();

	// Rebuild path
	std::vector<graph::node> result;

	current = goal;
	while(current != start) {
		result.push_back(g.get_node(current));
		current = prev[current];
	}
	result.push_back(g.get_node(start));
	std::reverse(result.begin(), result.end());

	return result;
}

