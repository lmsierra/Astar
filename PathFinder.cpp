#include "PathFinder.h"
#include "MapNode.h"

AStarRequest PathFinder::CreateRequestAStar(const MapNode& origin, const std::vector<const MapNode*>& goal_nodes, const HeuristicFunction& hf)
{
	AStarRequest request = AStarRequest(goal_nodes, hf);

	// Init First Node
	request.m_unvisited_pq.insert(std::make_pair(0.0f, &origin));
	request.m_cost_map[&origin] = 0.0f;

	return request;
}

AStarRequest PathFinder::CreateRequestDijkstra(const MapNode& origin, const std::vector<const MapNode*>& goal_nodes)
{
	return CreateRequestAStar(origin, goal_nodes, nullptr);
}

bool PathFinder::Process(AStarRequest& request, unsigned int num_steps)
{
	if (request.m_goal_nodes.size() == 0)
		return false;

	unsigned int step_count = 0;

	while (request.m_unvisited_pq.size() > 0 && (num_steps == 0 || (num_steps > 0 && step_count < num_steps)))
	{
		++step_count;

		const PQPair   pair = *request.m_unvisited_pq.begin();
		const MapNode* node = pair.second;

		if (std::find(request.m_goal_nodes.begin(), request.m_goal_nodes.end(), node) != request.m_goal_nodes.end())
		{
			request.goal_reached = const_cast<MapNode*>(node);
			return true;
		}

		request.m_unvisited_pq.erase(request.m_unvisited_pq.begin());
		request.m_visited_list.push_back(node);

		// Process links:
		const std::vector<MapNode*>& links = node->GetLinks();
		for (const MapNode* link : links)
		{
			// Not in closed list.
			if (std::find(request.m_visited_list.begin(), request.m_visited_list.end(), link) != request.m_visited_list.end())
				continue;

			const float cost     = request.m_cost_map[node] + static_cast<float>(Vector3D::Dist(link->GetPosition(), node->GetPosition()));
			const float estimate = request.m_heuristic ? request.m_heuristic(*link, request.m_goal_nodes) : 0.0f;
			const float value    = cost + estimate;

			// Look if the cost is lower than the previously stored (if any) or if the link doesn't exists in the open list.
			if (request.m_cost_map.find(link) == request.m_cost_map.end() || request.m_cost_map[link] > value)
			{
				request.m_cost_map[link] = value;
				request.m_reverse_path[const_cast<MapNode*>(link)] = const_cast<MapNode*>(node);

				for (const PQPair& n : request.m_unvisited_pq)
				{
					if (n.second == link)
						continue;
				}
				request.m_unvisited_pq.insert(std::make_pair(value, link));
			}
		}
	}

	return false;
}

bool AStarRequest::BuildPath(std::vector<MapNode*>& out_path) const
{
	out_path.clear();

	if (!IsComplete())
		return false;

	if (m_goal_nodes.size() == 0) // no goals, we can't build a path
		return true;

	auto& reversePath = m_reverse_path;
	out_path.push_back(goal_reached);

	auto it = reversePath.find(goal_reached);
	while (it != reversePath.end())
	{
		out_path.insert(out_path.begin(), const_cast<MapNode*>(it->second));
		it = reversePath.find(it->second);
	}

	return true;
}
