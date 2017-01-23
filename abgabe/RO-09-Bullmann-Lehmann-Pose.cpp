#include "Pose.h"

Pose::Pose() {

}

Pose::Pose(double x, double y, int degree) {
  this->x = x;
  this->y = y;
  this->degree = degree;
}

double Pose::getX() const {
  return this->x;
}

double Pose::getY() const {
  return this->y;
}

int Pose::getDegree() const {
  return this->degree;
}

bool Pose::empty() {
  return this ? false : true;
}
