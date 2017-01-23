#include <set>
#include <vector>

#include "Node.h"

#ifndef ASTAR_H_
#define ASTAR_H_

class Astar {
  public:
    /**
     *  This constructor sets the inintial nodes, executes the A*-Algorithm and saves the resulting path in the private list ready to be retrieved.
     */
    Astar(Node startingNode, Node goalNode, std::set<Node> obstacles, int stepsize);
    /**
     *  For retrieving the calculated path.
     */
    std::vector<Node> getPath();
    bool hasPath();
    bool isGoalNode(Node nodeToCheck);
    void setPath();
  private:
    std::set<Node, Node::CompareNode> openList;
    std::set<Node, Node::CompareNode> closedList;
    std::vector<Node> path;
    Node goalNode;
    Node startingNode;
    std::set<Node> obstacles;
    Node lastNode;
    bool pathFound;
    bool isNodeAnObstacle(Node cur);
    void execute();
    void expandNode(Node & currentNode);
    void paintImage();
    int stepsize;
};

#endif
