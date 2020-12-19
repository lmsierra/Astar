# A* search algorithm

Simple implementation of an A* search algorithm in C++11. The search can be paused and resume based on steps, so different frames can be used for completing it, and it's thread safe.

## Description
The algorithm must find the path between an origin and the closest node from list of possible goals. 

Starting at the origin, it will visit the adjacent nodes taking priority for the one that has a lower cost to reach. This cost is going to be calculated as the actual cost for reaching the node (distance in this case) and an heuristic (an estimation of the cost from the node to the solution).

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/commons/5/5d/Astar_progress_animation.gif" alt="A* example"/>
</p>

## Dijkstra algorithm
The Dijkstra algorithm is a specialization of the A* where no heuristic is applied.  

## Implementation details
The search is done in a map formed by nodes that are linked between them, and the weight used is just the distance between their positions.

The heuristic function can be set when requesting a search, so a better function than the default one could improve the performance of the search.

Two different lists are used for grouping the nodes: an open list that stores the unvisited nodes and a close list forkeeping track of the nodes already processed.

The open list is optimized by using a priority queue (std::set is used for this purpose), so the algorithm always processes first the nodes with a lower cost.

Two hash maps are used: one for storing the total cost to reach to a node, and another one to keep track from what node we reached. This last one is used for generating the path from the goal to the origin.

All the information required for solving the algorithm is contained under the request objects and no information about the map is modified, so different problems can be solved safely from different threads.

## Possible improvements
* Instead of returning a request object to be handled by the caller, use proper handle objects.

* Set a maximum time limit for processing in the same way steps are used.
