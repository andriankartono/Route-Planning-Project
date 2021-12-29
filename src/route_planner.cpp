#include "route_planner.h"
#include <algorithm>
#include<iostream>

using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    //get the closest node to the user defined coordinates and use that node as start/end node
    start_node= &m_Model.FindClosestNode(start_x, start_y);
    end_node= &m_Model.FindClosestNode(end_x, end_y);
}


//Calculate the Heuristic Value which uses the distance from the end_node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    //*end_node to dereference the pointer n_node
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    //populate current_node.neighbors
    current_node->FindNeighbors();

    //traverse each node.
    //set parent, h value and g value
    //add the neighbor node to the open_list and set the visited attribute to true
    for(auto i : current_node->neighbors){
        i->h_value= CalculateHValue(i);
        i->g_value= current_node->g_value + current_node->distance(*i);
        i->parent= current_node;

        open_list.push_back(i);
        i->visited=true;
    }
}


//Defined for the sort function
bool Compare(RouteModel::Node *node1, RouteModel::Node *node2){
    float f1=node1->h_value + node1->g_value;
    float f2=node2->h_value + node2->g_value;

    return f1>f2;
}

//Sort the Nodes based on the sum of h and g value. The node with the biggest value is at the beginning of the list.
//returns the node with the lowest sum of h and g value to be used as the next node.
//Also remove the next node from the open list.
RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), Compare);
    auto temp_pointer = open_list.back();
    open_list.pop_back();
    return temp_pointer;
}


//Traverse through the chain of parents of nodes until the starting node.
//Add all of the traversed node into the path_found list, which is the whole path from beggining to end.
//Calculates the total distance traversed and save it to the variable distance which can later be accessed to get the total distance travelled.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    
    while(current_node!=start_node){
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node=current_node->parent;
    }
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    //Set current_node to the start_node to begin the AStarSearch process.
  	start_node->visited=true;
    current_node=start_node;
  	//open_list.push_back(start_node);

    //Add the neighbors of the current node to the open list(AddNeighbors) and choose the next node(NextNode) until our current_node is our end_node
    while(current_node!=end_node){
      	AddNeighbors(current_node);
        current_node= NextNode();
        //AddNeighbors(current_node);    	
    }
  
    //exits the while loop when we reach the end destination. After we reach the end destination reconstruct the whole path from start to end using the ConstructFinalPath Function.
  	m_Model.path=ConstructFinalPath(current_node);
}
