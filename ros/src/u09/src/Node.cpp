#include "Node.h"

Node::Node() {

}

Node::Node(double x, double y, int degree) {
  Node(Pose(x, y, degree));
}

Node::Node(Pose position) {
  this->position = position;
}

double Node::getX() const {
  return this->position.getX();
}

double Node::getY() const {
  return this->position.getY();
}

int Node::getDegree() const {
  return this->position.getDegree();
}

double Node::getCostSoFar() const {
  return this->cost;
}

void Node::setCostSoFar(double costSoFar) {
  this->cost = costSoFar;
}

double Node::getHeuristicEstimate() const {
  return this->estimate;
}

void Node::setHeuristicEstimate(double estimate) {
  this->estimate = estimate;
}

double Node::getCompleteCost() {
  return this->estimate + this->cost;
}

bool Node::empty() {
  return this ? false : true;
}

Paint Node::getPaint() {
  return this->paint;
}

void Node::setPaint(Paint paint) {
  this->paint = paint;
}

bool Node::hasParent() {
  return this->parent.empty();
}

Pose Node::getParent() {
  return this->parent;
}

void Node::setParent(Pose parent) {
  this->parent = parent;
}
