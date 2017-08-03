#ifndef AIRSIM_ROS_GRAPH_H
#define AIRSIM_ROS_GRAPH_H

#include <unordered_map>
#include <set>
#include <vector>
#include <limits>

class graph
{
public:
	using node_id = int;
    static node_id invalid_id() { return std::numeric_limits<node_id>::max(); }

	struct node
	{
		double x, y, z;
		node_id id;
        node_id parent; // used for RRT and any other tree structures
	};

	struct edge
	{
		node_id n1, n2;
		double cost;

		bool operator< (const struct edge& e) const {return n1 < e.n1;}
		bool operator> (const struct edge& e) const {return n1 > e.n1;}
	};

	node_id add_node(const double& x, const double& y, const double& z);
	node_id add_node(const double& x, const double& y, const double& z, node_id id);
	node_id add_node(node n);
	struct node& get_node(node_id id);

	void connect(node_id n1, node_id n2, double edge = 0);
	bool is_adjacent (node_id n1, node_id n2);
	double cost_of_edge (node_id n1, node_id n2);

	std::set<struct edge> adjacent_edges(node_id id);
	std::set<node_id> adjacent_nodes(node_id id);

	int size();

	std::set<node_id> node_ids () const;

private:
	std::unordered_map<node_id, node> nodes;
	std::unordered_map<node_id, std::set<struct edge> > adj_list;
};

#endif
