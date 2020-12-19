#include "MapNode.h"
#include "PathFinder.h"
#include "Vector3D.h"

static std::vector<MapNode*> s_map_nodes;
static PathFinder s_path_finder;

inline void LinkNodes(MapNode* n1, MapNode* n2)
{
    n1->AddLink(n2);
    n2->AddLink(n1);
}

void InitMap()
{
    s_map_nodes.push_back(new MapNode("0", Vector3D(50, 550, 0)));
    s_map_nodes.push_back(new MapNode("1", Vector3D(50, 550, 0)));
    s_map_nodes.push_back(new MapNode("2", Vector3D(300, 550, 0)));
    s_map_nodes.push_back(new MapNode("3", Vector3D(500, 550, 0)));
    s_map_nodes.push_back(new MapNode("4", Vector3D(300, 425, 0)));
    s_map_nodes.push_back(new MapNode("5", Vector3D(175, 300, 0)));
    s_map_nodes.push_back(new MapNode("6", Vector3D(100, 150, 0)));
    s_map_nodes.push_back(new MapNode("7", Vector3D(450, 150, 0)));
    s_map_nodes.push_back(new MapNode("8", Vector3D(650, 400, 0)));
    s_map_nodes.push_back(new MapNode("9", Vector3D(550, 200, 0)));
    s_map_nodes.push_back(new MapNode("10", Vector3D(725, 325, 0)));

    LinkNodes(s_map_nodes[1], s_map_nodes[2]);
    LinkNodes(s_map_nodes[2], s_map_nodes[3]);
    LinkNodes(s_map_nodes[2], s_map_nodes[4]);
    LinkNodes(s_map_nodes[2], s_map_nodes[5]);
    LinkNodes(s_map_nodes[3], s_map_nodes[7]);
    LinkNodes(s_map_nodes[3], s_map_nodes[8]);
    LinkNodes(s_map_nodes[4], s_map_nodes[5]);
    LinkNodes(s_map_nodes[4], s_map_nodes[9]);
    LinkNodes(s_map_nodes[5], s_map_nodes[6]);
    LinkNodes(s_map_nodes[5], s_map_nodes[7]);
    LinkNodes(s_map_nodes[5], s_map_nodes[8]);
    LinkNodes(s_map_nodes[6], s_map_nodes[7]);
    LinkNodes(s_map_nodes[8], s_map_nodes[10]);
    LinkNodes(s_map_nodes[9], s_map_nodes[10]);
}

int main()
{
    InitMap();
    AStarRequest path_request = s_path_finder.CreateRequestAStar(*s_map_nodes[1], { s_map_nodes[7], s_map_nodes[10] });

    int steps = 0;
    while (!path_request.IsComplete())
    {
        ++steps;
        s_path_finder.Process(path_request, 1);

        std::cout << "----------------- Step: " << steps << "-----------------\n";
        if(path_request.m_visited_list.size() - 1 <= path_request.m_cost_map.size())
            std::cout << "Accumulated Cost: " << path_request.m_cost_map[path_request.m_visited_list[path_request.m_visited_list.size() - 1]] << "\n";
        std::cout << "Visited: \n\t";
        for (const MapNode* node : path_request.m_visited_list)
        {
            std::cout << node->GetName() << "  |  ";
        }
        std::cout << "\nToVisit:\n\t";
        for (const PQPair& p : path_request.m_unvisited_pq)
        {
            std::cout << p.second->GetName() << "(" << p.first << ")  |  ";
        }

        std::cout << "\n";
    }

    std::vector<MapNode*> path;
    path_request.BuildPath(path);

    std::cout << "Path found in " << steps << "steps\n";
    for (MapNode* node : path)
    {
        std::cout << node->GetName() << "  -  ";
    }

    return 0;
}
