#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

//generic C/C++ include
#include <string>
#include <sstream>
#include <ros/console.h>

const double l1 = 1.0;
const double l2 = 0.9;
double theta1 = -M_PI * 1.0 / 8.0;
double theta2 = M_PI * 3.0 / 4.0;
double oldX = -0.5;
double oldY = 0.5;
ros::Publisher pub_datainfo_joints;
ros::Publisher pub_oldX;
ros::Publisher pub_oldY;
std_msgs::Float64 msg_coords;

void inverseTF(double dx, double dy, double *dtheta1, double *dtheta2) {
  const double j11 = -oldY;
  const double j12 = -l2 * cos(theta1 + theta2);
  const double j21 = oldX;
  const double j22 = -l2 * sin(theta1 + theta2);
  const double abs = j11*j22 - j12*j21;

  *dtheta1 = (j22*dx - j12*dy) / abs;
  *dtheta2 = (j11*dy - j21*dx) / abs;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "arm2r_tf_listener");
  
  int iterations = atoi(argv[1]);

  ros::NodeHandle node;

  sensor_msgs::JointState jointstate_msg;
  jointstate_msg.name.push_back("joint_base_first");
  jointstate_msg.name.push_back("joint_first_second");
  //reset the messages
  // jointstate_msg.position.clear();
  // jointstate_msg.velocity.clear();
  // jointstate_msg.effort.clear();
  pub_datainfo_joints=node.advertise<sensor_msgs::JointState>(node.resolveName("/calibrated/joint_states"), 1);

  tf::TransformListener listener;
  
  jointstate_msg.header.stamp = ros::Time(0);
  jointstate_msg.position.push_back(theta1);
  //jointstate_msg.position[0]=1;
  jointstate_msg.position.push_back(theta2);
  //jointstate_msg.position[1]=0.5;
  pub_datainfo_joints.publish(jointstate_msg);

  msg_coords.data = oldX;
  pub_oldX.publish(msg_coords);
  msg_coords.data = oldY;
  pub_oldY.publish(msg_coords);

  double xCoords[] = {-0.5, 0.5, 0.5, -0.5};
  double yCoords[] = {-0.5, -0.5, 0.5, 0.5};

  double rateI = std::min(32.0, std::max(1.0, iterations / 8.0));
  ros::Rate rate(rateI);

  for (int line = 0; line < 4; line++) {
    int step = 0;
    double targetXCoord = xCoords[step];
    double targetYCoord = yCoords[step];
    while (ros::ok() && step < iterations) {
      tf::StampedTransform transform;
      try {
        //listener.waitForTransform("/second_link", "/base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/second_link", "/base_link", ros::Time(0), transform);

        oldX = transform.getOrigin().x();
        oldY = transform.getOrigin().y();
        double curStep = step++ / iterations;
        double dtheta1, dtheta2;
        inverseTF(curStep * (targetXCoord-oldX), curStep * (targetYCoord-oldY), &dtheta1, &dtheta2);
        theta1 += dtheta1;
        theta2 += dtheta2;

        jointstate_msg.header.stamp = ros::Time(0);
        jointstate_msg.position.push_back(theta1);
        //jointstate_msg.position[0]=1;
        jointstate_msg.position.push_back(theta2);
        //jointstate_msg.position[1]=0.5;
        pub_datainfo_joints.publish(jointstate_msg);
        msg_coords.data = oldX;
        pub_oldX.publish(msg_coords);
        msg_coords.data = oldY;
        pub_oldY.publish(msg_coords);

        ROS_INFO("Translation = %f,%f,%f", transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
        rate.sleep();
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
      }
    }
  }
  return 0;
};
