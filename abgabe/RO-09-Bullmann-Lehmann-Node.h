#ifndef NODE_H_
#define NODE_H_

#include "Pose.h" 

enum Paint {OPEN, CLOSED, START, GOAL, PATH, OBSTACLE};

class Node {
  public:
    Node();
    Node(double x, double y, int degree);
    Node(Pose position);
    bool empty();
    double getX() const;
    double getY() const;
    int getDegree() const;
    double getCostSoFar() const;
    void setCostSoFar(double costSoFar);
    double getHeuristicEstimate() const;
    void setHeuristicEstimate(double estimate);
    double getCompleteCost();
    Paint getPaint();
    void setPaint(Paint paint);
    bool hasParent();
    Pose getParent();
    void setParent(Pose parent);
    bool operator == (const Node& other) const {
      return this->position.getX() == other.getX() and this->position.getY() == other.getY() and this->position.getDegree() == other.getDegree();
    }
    bool operator < (const Node& other) const {
      return (this->position.getX() < other.getX() or this->position.getY() < other.getY()) or (this->position.getX() == other.getX() and this->position.getY() == other.getY() and this->position.getDegree() < other.getDegree());
    }
    struct CompareNode {
      bool operator() (Node const & node1, Node const & node2) const {
        if (node1.getX() < node2.getX()) {
          return true;
        } else if (node1.getX() == node2.getX()) {
          if (node1.getY() < node2.getY()) {
            return true;
          } else if (node1.getY() == node2.getY()) {
            return node1.getDegree() < node2.getDegree();
          }
        } else {
          return false;
        }
      }
    };
    struct CompareCostWithHeuristic {
      bool operator() (Node const & node1, Node const & node2) const {
        if ((node1.getCostSoFar() + node1.getHeuristicEstimate()) < (node2.getCostSoFar() + node2.getHeuristicEstimate())) {
          return true;
        } else {
          return false;
        }
      }
    };
  private:
    double cost, estimate;
    Paint paint;
    Pose position, parent;
};

#endif
