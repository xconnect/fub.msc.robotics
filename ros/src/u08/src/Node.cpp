#include "Node.h"

Node::Node() {

}

Node::Node(Parent parent) {
  Node(parent.x, parent.y);
}

Node::Node(int x, int y) {
  this->x = x;
  this->y = y;
}

int Node::getX() const {
  return this->x;
}

int Node::getY() const {
  return this->y;
}

int Node::getCostSoFar() const {
  return this->cost;
}

void Node::setCostSoFar(int costSoFar) {
  this->cost = costSoFar;
}

int Node::getHeuristicEstimate() const {
  return this->estimate;
}

void Node::setHeuristicEstimate(int estimate) {
  this->estimate = estimate;
}

int Node::getCompleteCost() {
  return this->estimate + this->cost;
}

bool Node::empty() {
  return this ? false : true;
}

bool Parent::empty() {
  return this ? false : true;
}

Paint Node::getPaint() {
  return this->paint;
}

void Node::setPaint(Paint paint) {
  this->paint = paint;
}

Parent Node::getParent() {
  return this->parent;
}

bool Node::hasParent() {
  return this->parent.empty();
}

void Node::setParent(Parent parent) {
  this->parent = parent;
}
