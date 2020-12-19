#pragma once

#include <stdint.h>
#include <vector>
#include <set>
#include <utility>
#include <queue>
#include <unordered_map>
#include <assert.h>
#include <iostream>
#include <utility>
#include <set>
#include <limits>
#include <math.h> 
#include <functional>

#include "MapNode.h"
#include "Vector3D.h"

using PQPair            = std::pair<float, const MapNode*>;
using HeuristicFunction = std::function<float(const MapNode&, const std::vector<const MapNode*>&)>;

struct AStarRequest
{
	AStarRequest(const std::vector<const MapNode*>& goal_nodes, const HeuristicFunction& hf) :
		m_goal_nodes (goal_nodes), goal_reached(nullptr), m_heuristic(hf)
	{
		const int default_size = 32;
		m_visited_list.reserve(default_size);
		m_cost_map.reserve(default_size);
		m_reverse_path.reserve(default_size);
	}

	const std::vector<const MapNode*>                  m_goal_nodes;
	std::vector<const MapNode*>                        m_visited_list;
	std::set<PQPair>								   m_unvisited_pq;
	std::unordered_map<const MapNode*, float>          m_cost_map;
	std::unordered_map<const MapNode*, const MapNode*> m_reverse_path;
	const HeuristicFunction  						   m_heuristic;
	MapNode* goal_reached;

	bool BuildPath(std::vector<MapNode*>& out_path) const;

	inline bool IsComplete() const
	{
		return goal_reached != nullptr || m_goal_nodes.size() == 0;
	}
};

class PathFinder
{
public:
	PathFinder() {}
	~PathFinder() {}

	AStarRequest CreateRequestAStar    (const MapNode& origin, const std::vector<const MapNode*>& goal_nodes, const HeuristicFunction& hf = CalculateAStarHeuristic);
	AStarRequest CreateRequestDijkstra (const MapNode& origin, const std::vector<const MapNode*>& goal_nodes);
	
	bool Process(AStarRequest& request, unsigned int num_steps = 0);
	
private:
	inline float static CalculateAStarHeuristic(const MapNode& node, const std::vector<const MapNode*>& destinations)
	{
		float min = FLT_MAX;
		for (const MapNode* dest : destinations)
		{
			min = fmin(min, static_cast<float>(Vector3D::Dist(node.GetPosition(), dest->GetPosition())));
		}

		return FLT_MAX ? 0.0f : min;
	}
};
