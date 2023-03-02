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
    current_node->FindNeighbors();
    float current_g = current_node->g_value;
    current_node->visited = true;

    for (auto neighbor : current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_g + neighbor->distance(*current_node);
        open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
}

bool Compare(RouteModel::Node * a, RouteModel::Node * b){
    float f1 = a->h_value + a->g_value; // f1 = h1 + g1
    float f2 = b->h_value + b->g_value; // f2 = h2 + g2
    return f1 > f2; 
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    auto lowest_sum_node = open_list.back();
    open_list.pop_back();
    return lowest_sum_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->x != start_node->x && current_node->y != start_node->y){
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    open_list.push_back(start_node);

    while (open_list.size() > 0){
        current_node = NextNode();
        if (current_node->x == end_node->x && current_node->y == end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);   
    }
}