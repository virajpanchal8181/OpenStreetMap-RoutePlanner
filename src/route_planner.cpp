#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

  
    // Store the nodes found in the RoutePlanner's start_node and end_node attributes.
   	start_node = &m_Model.FindClosestNode(start_x, start_y);
  	end_node = &m_Model.FindClosestNode(end_x, end_y); 

}


// Implement the CalculateHValue method using distance() of RouteModel::Node

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  	
  	float H_value = 0.0f;
  	H_value = node->distance(*end_node); //Calculate distance to the end_node for the h value
  	return H_value;
}


//  AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  	
  	current_node->FindNeighbors();   //populate current_node.neighbors vector with all the neighbors
  
  	for(RouteModel::Node *neighbor : current_node->neighbors) {
      	neighbor->parent = current_node;
      	neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
      	open_list.push_back(neighbor); 
        neighbor->visited = true;
    }
}


// NextNode method to sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {
  	
    //Sort the open_list according to the sum of the h value and g value in descending order.
  	std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *node1, const RouteModel::Node *node2){
      												return (node1->g_value + node1->h_value) > (node2->g_value + node2->h_value);});
      																										
  	auto next_node = open_list.back(); 
  	open_list.pop_back();
  	return next_node;
}


// ConstructFinalPath method to return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *node = current_node;
  	while(node->parent != nullptr){
        distance += node->distance(*(node->parent));
      	path_found.push_back(*node);
      	node = node->parent;
    }
    
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());   //corrects the order of the path from start to end node   		

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    
    return path_found;
}


// A* Search algorithm to find the shortest path available

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    // TODO: Implement your solution here.
    current_node = start_node;
    current_node->visited = true;
    std::vector<RouteModel::Node>path_created;
    open_list.push_back(current_node);
    
    while(open_list.size() > 0){
        
        AddNeighbors(current_node);
        auto next_node = NextNode();
        
      	if(next_node == end_node){            
           path_created = ConstructFinalPath(next_node);
           break;
        }
        
       current_node = next_node;      
    }    
   m_Model.path = path_created;
}
