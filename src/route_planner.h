#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"
using std::vector;

class RoutePlanner
{
public:
  RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
  // Add public variables or methods declarations here.
  float GetDistance() const
  {
    return distance;
  }
  void AStarSearch();

private:
  // Add private variables or methods declarations here.
  RouteModel &m_Model;
  RouteModel::Node *start_node = nullptr;
  RouteModel::Node *end_node = nullptr;
  float distance = 0.0; //will hold the total distance for the route that A* search finds from start_node to end_node
  vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node * current_node);
};

#endif