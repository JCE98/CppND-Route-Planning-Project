#include "route_planner.h"
#include <algorithm>

using namespace std;

//Route Planner Constructor
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes (defined in route_planner.h).
    /*
    start_node and end_node are attributes of the RoutePlanner::Routplanner class initialized with this constructor (defined in route_planner.h), 
    so they can be accessed directly.
    m_Model is the RouteModel object, with method FindClosestNode, as defined in route_model.h
    */
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    /* The pointer to a node that is input to this method must be dereferenced to access it's distance method (defined in route_model.h)
    Similarly, the reference point node (end_node in this case) must be dereferenced, since it's a pointer attribute of the RoutePlanner object 
    (defined in route_planner.h)
    */
    return node->distance(*end_node);     //Distance to the end node is the heuristic score. This will be combined with distance to neighbor in slecting next nodes

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // The pointer to the current node is used to access it's FindNeighbors method, as defined in route_model.h
    current_node->FindNeighbors();
    // This is a range based loop on a vector of pointers to the current_node neighbors, populated by the method used above
    for (RouteModel::Node *neighbor : current_node->neighbors)
    {
        // The attributes of each neighbor can be set using the pointers to the neighbor nodes
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);  //This is the total distance from the start_node, building on the parent node
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    /* The open_list is a vector of pointers to nodes. The h_value and g_value are attributes of each RouteModel::Node object (defined in route_model.h)
    A lambda function is a concise way to perform an operation accross a vector, which is what will be required for the sort method
    The inputs to the lambda function should be pointers to nodes, since that is the target type of the items in open_list
    The pointer to the last element of the open_list (the lowest f-score) should be a RouteModel::Node object, since open_list is a vector of pointers 
    to these objects
    */
    sort(open_list.begin(),open_list.end(),[](const RouteModel::Node *node1, const RouteModel::Node *node2)->float\
        {return node1->h_value+node1->g_value > node2->h_value+node2->g_value;});       //Sort the open_list according to f-score
    RouteModel::Node *lowest_node = open_list.back();           //Extract the node with the lowest f-score
    open_list.pop_back();                                       //Remove the selected next node from the open_list, so it cannot be selected again
    return lowest_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    /*This is the definition of a RoutePlanner class method declared in the route_planner.h header file
    As such, all of the attributes of a RoutePlanner object are available directly
    */
    while (current_node != start_node)
    {
        path_found.insert(path_found.begin(),*current_node);       //Insert each node at the beginning of the path_found vector of nodes for start to finish order
        distance += current_node->distance(*current_node->parent); //Increment the distance attribute of RouteModel using RouteModel::Node::distance to the parent node
        current_node = current_node->parent;                       //Change the current node to the parent of the current node for the next loop iteration
    }
    path_found.insert(path_found.begin(),*current_node);           // Insert the start node as the first entry once path finding is complete

    distance *= m_Model.MetricScale();                             // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

/* The current node is initialized to the start node, which is accessible directly because AStarSearch is a method in the RoutePlanner parent class.
    The visited flag and h_value are also calculated for this node, to enable the calculations for subsequent nodes. The g_value is initialized in the 
    class definition.
    The next node to explore is determined using the NextNode method, combining the heuristic score with the distance from the current node to the neighbor as 
    a potential next node.
    The while loop is an infinite loop, with a break statement accessed by the convergence of the current_node to the end_node.
    The neighbors of each node are added to the open_list of nodes using the AddNeighbors method.
    The ConstructFinalPath method is used to work backwards from the current_node (now the end_node once the loop exits) by following the chain of parent nodes back
    to the start_node.
*/
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    current_node = start_node;                                              //Initialize the current node to the start node
    current_node->visited = true;                                           //Mark this node as visited
    open_list.push_back(current_node);                                      //Add the start_node to the open_list. This node has a high heuristic score.
    while (open_list.size()>0)                                              //Inifinite while loop, broken by reaching the end_node
    {
        current_node = NextNode();                                          //Identify which of the current_node neighbors to explore next
        if (current_node == end_node)                                       //Infinite while loop break condition
        {
            m_Model.path = ConstructFinalPath(current_node);                //Construct complete path once end_node is reached by A* Search algorithm 
            return;
        }
        AddNeighbors(current_node);                                         //Add neighbors of current_node to open_list for evaluation as potential next nodes
    }
}