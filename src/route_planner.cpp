#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();
    for (auto &neighbor: current_node->neighbors) {
        // - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        // - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    // - Sort the open_list according to the sum of the h value and g value.
    std::sort(open_list.begin(), open_list.end(), [](const auto & _1st, const auto & _2nd){
        return _1st->h_value + _1st->g_value > _2nd->h_value + _2nd->g_value;
    });
    // - Create a pointer to the node in the list with the lowest sum.
    auto nextNode = open_list.back();
    // - Remove that node from the open_list.
    open_list.pop_back();
    // - Return the pointer.
    return nextNode;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // - This method should take the current (final) node as an argument and iteratively follow the 
    //   chain of parents of nodes until the starting node is found.
    while ((current_node->x != start_node->x) || (current_node->y != start_node->y)) {
        // - For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*current_node->parent);
        path_found.insert(path_found.begin(), *current_node);
        current_node = current_node->parent;
    }
    path_found.insert(path_found.begin(), *current_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    current_node->visited = true;
    open_list.push_back(current_node);
    while (open_list.size() > 0) {
        // - Use the NextNode() method to sort the open_list and return the next node.
        current_node = NextNode();
        // - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
        if (current_node->distance(*end_node) == 0) {
            // - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
        AddNeighbors(current_node);
    }
}