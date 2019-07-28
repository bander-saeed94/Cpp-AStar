#include "route_planner.h"
#include <algorithm>
using std::cout;
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    cout << "start_x: " << start_x << ", start_y: " << start_y << "\n";
    cout << "end_x: " << end_x << ", end_y: " << end_y << "\n";
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    cout << "start_x: " << start_x << ", start_y: " << start_y << "\n";
    cout << "end_x: " << end_x << ", end_y: " << end_y << "\n";

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    cout << "start_node->x: " << start_node->x << ", start_node->y: " << start_node->y << "\n";
    cout << "end_node->x: " << end_node->x << ", end_node->y: " << end_node->y << "\n";
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

