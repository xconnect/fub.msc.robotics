#ifndef NODE_H_
#define NODE_H_

enum Paint {OPEN, CLOSED, START, GOAL, PATH, OBSTACLE};

class Parent {
  public:
    int x, y;
    bool empty();
};

class Node {
  public:
    Node();
    Node(Parent parent);
    Node(int x, int y);
    bool empty();
    int getX() const;
    int getY() const;
    int getCostSoFar() const;
    void setCostSoFar(int costSoFar);
    int getHeuristicEstimate() const;
    void setHeuristicEstimate(int estimate);
    int getCompleteCost();
    Paint getPaint();
    void setPaint(Paint paint);
    bool hasParent();
    Parent getParent();
    void setParent(Parent parent);
    bool operator == (const Node& other) const {
      return this->x == other.getX() and this->y == other.getY();
    }
    bool operator < (const Node& other) const {
      return this->x < other.getX() or this->y < other.getY();
    }
    struct CompareNode {
      bool operator() (Node const & node1, Node const & node2) const {
        if (node1.getX() < node2.getX()) {
          return true;
        } else if(node1.getX() == node2.getX()) {
          return node1.getY() < node2.getY();
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
    int x,y, cost, estimate;
    Paint paint;
    Parent parent;
};

#endif
