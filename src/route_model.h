#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>
#include <vector>
using std::unordered_map;
using std::vector;

class RouteModel : public Model
{

public:
  class Node : public Model::Node
  {
  public:
    // Add public Node variables and methods here.
    Node *parent = nullptr;
    float h_value = std::numeric_limits<float>::max();
    float g_value = 0.0;
    bool visited = false;
    vector<Node *> neighbors;
    void FindNeighbors();

    float distance(const Node node) const
    {
      float xDiff = (this->x - node.x);
      float xDiffPower2 = xDiff * xDiff;
      float yDiff = (this->y - node.y);
      float yDiffPower2 = yDiff * yDiff;
      return std::sqrt(xDiffPower2 + yDiffPower2);
    }
    Node() {}
    Node(int idx, RouteModel *search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

  private:
    // Add private Node variables and methods here.
    int index;
    RouteModel *parent_model = nullptr;
    RouteModel::Node *FindNeighbor(vector<int> node_indices);
  };

  // Add public RouteModel variables and methods here.
  RouteModel(const vector<std::byte> &xml);
  RouteModel::Node &FindClosestNode(int x, int y);
  vector<Node> path; // This variable will eventually store the path that is found by the A* search.
  vector<Node> &SNodes() { return m_Nodes; }
  unordered_map<int, vector<const Model::Road *>> &GetNodeToRoadMap()
  {
    return node_to_road;
  }

private:
  // Add private RouteModel variables and methods here.
  vector<Node> m_Nodes;
  unordered_map<int, vector<const Model::Road *>> node_to_road;
  void CreateNodeToRoadHashmap();
};

#endif