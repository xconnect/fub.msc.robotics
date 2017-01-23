#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>

int t = 0;
ros::Subscriber subscriberLaser;
ros::Publisher publishXCoord;
ros::Publisher publishSpeed;
std_msgs::Float64 msg;
Eigen::Matrix2f a;
Eigen::Matrix2f h;
Eigen::Matrix2f p;
Eigen::Matrix2f q;
Eigen::Matrix2f r;
Eigen::Vector2f x;

void laserCB(const sensor_msgs::LaserScan &laser) {
  Eigen::Vector2f m, x2, x3;
  Eigen::Matrix2f i, k, p2, p3;
  int time = laser.header.stamp.nsec;
  m(0) = laser.ranges[269] + 0.5;
  m(1) = 1000000000 * (m(0) - x(0, 0)) / (time - t);
  x2 = a * x;
  p2 = a * p * a.transpose() + q;
  i = h * p2 * h.transpose();
  k = p2 * h.transpose() * i.inverse();
  x3 = x2 + k * (m - h * x2);
  p3 = (Eigen::Matrix2f::Identity() - k * h) * p2;
  t = time;
  x = x3;
  p = p3;
  msg.data = x(0);
  publishXCoord.publish(msg);
  msg.data = x(1);
  publishSpeed.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kalman");
  ros::Time::init();
  ros::Rate rr(10);
  ros::NodeHandle n;
  x << 0,
    0;
  p << 1000, 0,
    0,    1000;
  a << 1, 0.04,
    0, 1;
  h << 1, 0,
    0, 1;
  q << 1, 0,
    0, 1;
  r << 1, 0,
    0, 1;
  subscriberLaser = n.subscribe("/ackerman_vehicle/laser/scan", 10, laserCB);
  publishSpeed = n.advertise<std_msgs::Float64>("/kalman_speed", 10);
  publishXCoord = n.advertise<std_msgs::Float64>("/kalman_x_coord", 10);
  ros::Duration(10).sleep();

  while(ros::ok()) {
    ros::spinOnce();
    rr.sleep();
  }

  return 0;
}
