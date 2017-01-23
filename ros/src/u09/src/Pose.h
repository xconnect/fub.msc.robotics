#ifndef POSE_H_
#define POSE_H_

class Pose {
  public:
    Pose();
    Pose(double x, double y, int degree);
    bool empty();
    double getX() const;
    double getY() const;
    int getDegree() const;
    bool operator == (const Pose& other) const {
      return this->x == other.getX() and this->y == other.getY() and this->degree == other.getDegree();
    }
    struct ComparePose {
      bool operator() (Pose const & pose1, Pose const & pose2) const {
        if (pose1.getX() < pose2.getX()) {
          return true;
        } else if (pose1.getX() == pose2.getX()) {
          if (pose1.getY() < pose2.getY()) {
            return true;
          } else if (pose1.getY() == pose2.getY()) {
            return pose1.getDegree() < pose2.getDegree();
          }
        } else {
          return false;
        }
      }
    };
  private:
    double x,y;
    int degree;
};

#endif
