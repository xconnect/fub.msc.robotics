#include "ros/ros.h"
#include "arm2r/ChangePose.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm2r_client");
  if (argc != 2) {
    ROS_INFO("usage: '1' for positioning the roboarm to the left, '2' for the starting position and '3' to position the arm to the right.");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<arm2r::ChangePose>("change_pose");
  arm2r::ChangePose srv;
  srv.request.pose = atoi(argv[1]);
  if (client.call(srv)) {
    ROS_INFO("Success");
  } else {
    ROS_ERROR("Failed to call service change_pose");
    return 1;
  }

  return 0;
}
