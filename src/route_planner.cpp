#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

	start_node = &m_Model.FindClosestNode(start_x, start_y);//used reference to m_Model to avoid impossible conversion from node to node*
  	end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	float h_value = (*node).distance(*end_node); //distance from node to the end_node
    return h_value;//return heuristic
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

  	(*current_node).FindNeighbors();//find current nodes neighbors
    for(RouteModel::Node* node: (*current_node).neighbors){//loop through all of the neighbors
      (*node).parent = current_node;	//make the parent the current node
      (*node).h_value = CalculateHValue(node);//calculate the H_value
      (*node).g_value = (*current_node).g_value + (*node).distance(*current_node);//calc the g value
      open_list.push_back(node);//add new node to the open list
      (*node).visited = true;//mark as visited
    }
}

bool Compare(const RouteModel::Node* a, const RouteModel::Node* b){
 
	float f_value1 = (*a).g_value + (*a).h_value;//calc f values
    float f_value2 = (*b).g_value + (*b).h_value;
  	return f_value1 > f_value2;//sort in descending order.
}

RouteModel::Node *RoutePlanner::NextNode() {

 	sort(open_list.begin(), open_list.end(), Compare);//sort open_list by calling compare as the overload function
 	RouteModel::Node* p = open_list.back();//pointer to the lowest f value
 	open_list.pop_back();//remove that value
  	return p;// pass the pointer, the value is still there
  
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

  	RouteModel::Node* node_on = current_node;//node_on is now the final node
  	path_found.push_back(*node_on);//add the final node to the path
  	//distance += node_on.parent.distance(*node_on);
  	
  do{
   	  distance += node_on.parent.distance(*node_on);//add the distance from node_on to its parent
      node_on = (*node_on).parent;//make node_on its parent node to continue and loop through
      path_found.push_back(*node_on);//add new node (the parent) to the path
    
  }while(node_on != start_node);//while there are still parent nodes available
  	
  	reverse(path_found.begin(), path_found.end());//sort the vector to start at the "start"
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    open_list.push_back(start_node);//add the starting node to the open list
  	start_node.visited = true;
  	 while (open_list.size() > 0){//while there are nodes in the open list
       current_node = NextNode();//call NextNode function
       if(current_node == end_node){//if we are at the end
         m_Model.path = ConstructFinalPath(current_node);//construct the final path
         break;
       }
       else{
       AddNeighbors(current_node);//otherwise add the neighbors to the current node
       }
     }
}