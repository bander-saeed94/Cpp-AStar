#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, end_x);
    end_node = &m_Model.FindClosestNode(start_y, end_y);
}
vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node * current_node){
    vector<RouteModel::Node> path{};
    distance = 0.0;
    RouteModel::Node parent;

    while(current_node->parent != nullptr){
            path.push_back(*current_node);
            parent = *(current_node->parent);
            distance += parent.distance(*current_node);
            // distance = current_node->distance(parent);
            current_node = current_node->parent;
    }
    path.push_back(*current_node);
    distance *= m_Model.MetricScale();
    return path;
}

void RoutePlanner::AStarSearch(){
    end_node->parent = start_node;
    m_Model.path = ConstructFinalPath(end_node);
}

