#include "Astar.h"


#include <stdio.h>
#include <string.h>
#include "opencv2/opencv.hpp"

Astar::Astar(Node startingNode, Node goalNode, std::set<Node> obstacles, int stepsize) {
  pathFound = false;
  this->obstacles = obstacles;
  this->goalNode = goalNode;
  this->goalNode.setPaint(GOAL);
  this->startingNode = startingNode;
  this->startingNode.setPaint(START);
  this->stepsize = stepsize;
  startingNode.setCostSoFar(0);
  openList.insert(startingNode);
  execute();
  setPath();
  paintImage();
}

std::vector<Node> Astar::getPath() {
  return path;
}

bool Astar::hasPath() {
  return pathFound;
}

bool Astar::isGoalNode(Node nodeToCheck) {
  if ((abs(this->goalNode.getX() - nodeToCheck.getX()) + abs(this->goalNode.getY() - nodeToCheck.getY())) < 6.25
      and ((this->goalNode.getDegree() - nodeToCheck.getDegree()) > (this->goalNode.getDegree() - 60)
      or (this->goalNode.getDegree() - nodeToCheck.getDegree()) < (this->goalNode.getDegree() + 60))) {
    return true;
  }
  return false;
}

void Astar::paintImage() {
  int width  = 15;
  int height = 19;
  int s = 19;
  int r = s / 2;
  int o = r + 1;
  cv::Scalar white(255, 255, 255);
  cv::Scalar grey(200, 200, 200);
  cv::Scalar blue(200, 0, 0);
  cv::Scalar purple(200, 0, 200);
  cv::Scalar green(0, 200, 0);
  cv::Scalar red(0, 0, 200);
  cv::Scalar yellow(0, 255, 255);
  cv::Mat img(s*width, s*height, CV_8UC3, white);
  std::vector<Node> allNodes;
  for (std::set<Node>::iterator it = closedList.begin(); it != closedList.end(); it++) {
    allNodes.push_back(*it);
  }
  for (std::set<Node>::iterator it = openList.begin(); it != openList.end(); it++) {
    allNodes.push_back(*it);
  }
  for (std::set<Node>::iterator it = obstacles.begin(); it != obstacles.end(); it++) {
    allNodes.push_back(*it);
  }
  allNodes.push_back(this->goalNode);
  allNodes.push_back(this->startingNode);

  Node par;
  for (std::vector<Node>::iterator it = allNodes.begin(); it != allNodes.end(); it++) {
    Node cur = *it;
    int x = cur.getX();
    int y = cur.getY();
    switch(cur.getPaint()) {
      case OPEN:      cv::circle(img, cv::Point(s*x+o, s*y+o), r, grey, -1);
                      break;
      case CLOSED:    cv::circle(img, cv::Point(s*x+o, s*y+o), r, purple, -1);
                      break;
      case START:     cv::circle(img, cv::Point(s*x+o, s*y+o), r, red, -1);
                      break;
      case PATH:      par = Node(cur.getParent());
                      cv::circle(img, cv::Point(s*x+o, s*y+o), r/2, yellow, -1);
                      break;
      case GOAL:      cv::circle(img, cv::Point(s*x+o, s*y+o), r, blue, -1);
                      break;
      case OBSTACLE:  cv::circle(img, cv::Point(s*x+o, s*y+o), r, green, -1);
                      break;
    }
  }
  for (std::vector<Node>::iterator it = path.begin(); it != path.end(); it++) {
    Node cur = *it;
    int x = cur.getX();
    int y = cur.getY();
    cv::circle(img, cv::Point(s*x+o, s*y+o), r/2, yellow, -1);
  }
  std::set<Node>::iterator it = std::min_element(obstacles.begin(), obstacles.end(), Node::CompareNode());
  Node currentNode = *it;
  std::stringstream title, filepath;
  title << "i=" << path.size();
  cv::namedWindow(title.str());
  cv::imshow(title.str(), img);
  filepath << "./img/" << "ue9_t1-" << (this->pathFound ? "true" : "false") << ".png";
  cv::imwrite(filepath.str(), img);
  cv::waitKey(0);
}

void Astar::setPath() {
  path.clear();
  if (pathFound) {
    while (not lastNode.empty()) {
      lastNode.setPaint(PATH);
      path.push_back(lastNode);
      if (lastNode.hasParent()) {
//        lastNode = Node(lastNode.getParent());
        if (closedList.count(Node(lastNode.getParent())) == 1) {
          lastNode = * closedList.find(Node(lastNode.getParent()));
        }
      } else {
        return;
      }
    }
  }
}

void Astar::execute() {
  do {
    std::set<Node>::iterator it = std::min_element(openList.begin(), openList.end(), Node::CompareCostWithHeuristic());
    Node currentNode = *it;
    openList.erase(it);
    if (isGoalNode(currentNode)) {
      pathFound = true;
      lastNode = currentNode;
      return;
    } else {
      currentNode.setPaint(CLOSED);
      closedList.insert(currentNode);
    }
    expandNode(currentNode);
  } while (not openList.empty());
  pathFound = false;
}

void Astar::expandNode(Node & currentNode) {
  Node successorNode;
  for (int i = 0; i < 3; i++) {
    switch(i) {
      case 0: successorNode = Node(currentNode.getX()+1*stepsize, currentNode.getY()-1*stepsize, 45);
              break;
      case 1: successorNode = Node(currentNode.getX()+1*stepsize, currentNode.getY(), 0);
              break;
      case 2: successorNode = Node(currentNode.getX()+1*stepsize, currentNode.getY()+1*stepsize, -45);
              break;
    }
    if(std::find(obstacles.begin(), obstacles.end(), successorNode) != obstacles.end()) {
      //this node is an obstacle, so don't go there biatch
      continue;
    }
    if (closedList.count(successorNode) == 1) {
      continue;
    } else {
      //setting cost from parent plus way from parent to current
      successorNode.setCostSoFar(currentNode.getCostSoFar()+1);
      successorNode.setHeuristicEstimate(std::abs(goalNode.getX()-successorNode.getX())+std::abs(goalNode.getY()-successorNode.getY()));
      if (openList.count(successorNode) == 1) {
        Node oldSuccessorNode = *openList.find(successorNode);
        if (successorNode.getCompleteCost() >= oldSuccessorNode.getCompleteCost()) {
          continue;
        }
      }
      Pose temp = Pose(currentNode.getX(), currentNode.getY(), currentNode.getDegree());
      successorNode.setParent(temp);
      successorNode.setPaint(OPEN);
      if (openList.count(successorNode) == 1) {
        openList.erase(openList.find(successorNode));
      }
      openList.insert(successorNode);
    }
  }
}

int main(int, char**) {
  std::set<Node> obstacles;
  obstacles.clear();
  for(int i=-5; i<21; ++i) {
    for(int j=15; j<21; ++j) {
      Node temp = Node(j, i, 0);
      temp.setPaint(OBSTACLE);
      obstacles.insert(temp);
    }
  }
    Astar(Node(0, 0, 0), Node(30, 15, 0), obstacles, 2);
}
