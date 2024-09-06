#include "route_planner.h"
#include <algorithm>
#include <limits>
#include <unordered_set>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by
// relaxing all neighbors that are not closed.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - Update the parent, h_value, and g_value when a shorter route is found.
// - Use CalculateHValue below to implement the h-Value calculation.
// - Add newly discovered nodes to open_list. Nodes become visited only when
//   removed from the open list for expansion.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto node : current_node->neighbors) {
        const float tentative_g =
            current_node->g_value + current_node->distance(*node);
        if (tentative_g < node->g_value) {
            node->parent = current_node;
            node->h_value = CalculateHValue(node);
            node->g_value = tentative_g;
            if (std::find(open_list.begin(), open_list.end(), node) ==
                open_list.end()) {
                open_list.push_back(node);
            }
        }
    }
}

float FValue(const RouteModel::Node *node) {
    return node->g_value + node->h_value;
}

float PathDistance(const std::vector<RouteModel::Node> &path) {
    float dist = 0.0f;
    for (size_t i = 1; i < path.size(); i++) {
        dist += path[i].distance(path[i - 1]);
    }
    return dist;
}

bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
    if (FValue(a) != FValue(b)) return FValue(a) > FValue(b);
    return a->Index() > b->Index();
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    if (open_list.empty()) return nullptr;
    RouteModel::Node *current = nullptr;
    sort(open_list.begin(), open_list.end(), Compare);
    current = open_list.back();
    open_list.pop_back();
    return current;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    std::unordered_set<RouteModel::Node *> chain;
    while (current_node != start_node) {
        if (!current_node || !chain.insert(current_node).second) {
            distance = 0.0f;
            return {};
        }
        path_found.push_back(*current_node);
        if (!current_node->parent) {
            distance = 0.0f;
            return {};
        }
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    for (auto &node : m_Model.SNodes()) {
        node.visited = false;
        node.g_value = std::numeric_limits<float>::max();
        node.h_value = std::numeric_limits<float>::max();
        node.parent = nullptr;
        node.neighbors.clear();
    }
    // Clear any path left over from a prior search so a failed or
    // disconnected search does not report a stale result as current.
    m_Model.path.clear();
    distance = 0.0f;
    start_node->g_value = 0.0f;
    start_node->h_value = CalculateHValue(start_node);
    open_list.clear();
    open_list.push_back(start_node);
    while (!open_list.empty()) {
        // Get the next node
        current_node = NextNode();
        if (current_node->visited) continue;
        current_node->visited = true;
        // Check if we're done.
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(end_node);
            break;
        }
        AddNeighbors(current_node);
    }
}
