#include "route_planner.h"
#include <algorithm>
using std::cout;
using std::sort;
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    cout << "start_x: " << start_x << ", start_y: " << start_y << "\n";
    cout << "end_x: " << end_x << ", end_y: " << end_y << "\n";
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;


    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}
vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node * current_node){
    vector<RouteModel::Node> path{};
    distance = 0.0f;
    RouteModel::Node parent;

    while(current_node->parent != nullptr){
            path.push_back(*current_node);
            parent = *(current_node->parent);
            // distance += parent.distance(*current_node);
            distance += current_node->distance(parent);
            current_node = current_node->parent;
    }
    path.push_back(*current_node);
    distance *= m_Model.MetricScale();
    return path;
}

void RoutePlanner::AStarSearch(){
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;

    while (open_list.size() > 0)
    {
        current_node = NextNode();
        if(current_node->distance(*end_node) == 0){
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
    
    // end_node->parent = start_node;
    // m_Model.path = ConstructFinalPath(end_node);
}

float RoutePlanner::CalculateHValue(const RouteModel::Node * node){
    return node->distance(*end_node);
}

bool Compare(const RouteModel::Node * a, const RouteModel::Node * b) {
  int f1 = a->g_value + a->h_value; // f1 = g1 + h1
  int f2 = b->g_value + b->h_value; // f2 = g2 + h2
  return f1 > f2; 
}

RouteModel::Node * RoutePlanner::NextNode(){
    //Sort the open_list according to the f-value, which is the sum of a node's h-value and g-value.
    // sort(open_list.begin(), open_list.end(), Compare);
    // //Create a copy of the pointer to the node with the lowest f-value.
    // RouteModel::Node * copy_ptr = open_list.back();
    // //Erase that node pointer from open_list.
    // open_list.pop_back();
    // //Return the pointer copy.
    // return copy_ptr;
    sort(open_list.begin(), open_list.end(), [](const auto &_1st, const auto &_2nd){
        return _1st->g_value + _1st->h_value >  _2nd->g_value + _2nd->h_value;
    });
    RouteModel::Node * lowest_node = open_list.back();
    open_list.erase(open_list.begin());
    return lowest_node;
}
  void RoutePlanner::AddNeighbors(RouteModel::Node * current_node){
      current_node->FindNeighbors();
      for(auto neighbor : current_node->neighbors){
          neighbor->parent = current_node;
          neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
          neighbor->h_value = CalculateHValue(neighbor);
          open_list.push_back(neighbor);
          neighbor->visited = true;
      }
  }
