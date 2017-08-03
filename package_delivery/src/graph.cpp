#include "graph.h"
#include <limits>

graph::node_id graph::add_node(const double& x, const double& y, const double& z)
{
	return add_node(x, y, z, nodes.size());
}

graph::node_id graph::add_node(const double& x, const double& y, const double& z, graph::node_id id)
{
	nodes[id] = {x, y, z, id};
	return id;
}

graph::node_id graph::add_node(graph::node n)
{
	// return add_node(n.x, n.y, n.z, n.id);
	nodes[n.id] = n;
	return n.id;
}

struct graph::node& graph::get_node(graph::node_id id)
{
	return nodes[id];
}

void graph::connect(graph::node_id n1, graph::node_id n2, double cost)
{
	struct edge e = {n1, n2, cost};

	adj_list[n1].insert(e);
	adj_list[n2].insert(e);
}

bool graph::is_adjacent(graph::node_id n1, graph::node_id n2)
{
	if (adj_list.find(n1) == adj_list.end()) // if no nodes are adjacent to n1
		return false;

	auto edges = adj_list[n1];

	for (auto it = edges.begin(); it != edges.end(); ++it) {
		if (it->n1 == n2 || it->n2 == n2)
			return true;
	}

	return false;
}

std::set<struct graph::edge> graph::adjacent_edges(node_id id)
{
	if (adj_list.find(id) == adj_list.end())
		return std::set<struct edge>();

	return adj_list[id];	
}

std::set<graph::node_id> graph::adjacent_nodes(graph::node_id id)
{
	std::set<node_id> result;
	auto edges = adjacent_edges(id);

	for (auto it = edges.begin(); it != edges.end(); ++it) {
		if (it->n1 != id)
			result.insert(it->n1);
		else
			result.insert(it->n2);
	}

	return result;
}

int graph::size()
{
	return nodes.size();
}

std::set<graph::node_id> graph::node_ids() const
{
	std::set<graph::node_id> keys;
	for (auto it = nodes.begin(); it != nodes.end(); ++it) {
		keys.insert(it->first);
	}

	return keys;
}

double graph::cost_of_edge (graph::node_id n1, graph::node_id n2)
{
	if (adj_list.find(n1) == adj_list.end()) // if no nodes are adjacent to n1
		return std::numeric_limits<double>::quiet_NaN();;

	auto edges = adj_list[n1];

	for (auto it = edges.begin(); it != edges.end(); ++it) {
		if (it->n1 == n2 || it->n2 == n2)
			return it->cost;
	}

	return std::numeric_limits<double>::quiet_NaN();;
}
