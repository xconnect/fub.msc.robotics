#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "arm2r/ChangePose.h"

//generic C/C++ include
#include <string>

ros::Publisher pub_joints;
sensor_msgs::JointState msg_joints;

bool changePose(arm2r::ChangePose::Request &req, arm2r::ChangePose::Response &res)
{
	msg_joints.header.stamp = ros::Time::now();
  switch(req.pose) {
    case 1:
	    msg_joints.position[0]  = 0.5 * M_PI;
      break;
    case 2:
	    msg_joints.position[0]  = 0;
      break;
    case 3:
    	msg_joints.position[0]  = -0.5 * M_PI;
      break;
    default:
      ROS_ERROR("You gave a wrong position: '%li' is not viable. Please use '1' for left, '2' for base or '3' for right.", req.pose);
      res.success = 0;
      return false;
      break;
  }
	msg_joints.position[1]  = 0;
	pub_joints.publish(msg_joints);
	res.success = 1;
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "arm2r_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("change_pose", changePose);
	pub_joints = n.advertise<sensor_msgs::JointState>(n.resolveName("/joint_states_change"), 1);
	
  msg_joints.name.resize(2);
	msg_joints.position.resize(2);
	msg_joints.name[0] ="joint_base_first";
	msg_joints.name[1] ="joint_first_second";
	msg_joints.header.stamp = ros::Time::now();
	msg_joints.position[0]  = 0;
	msg_joints.position[1]  = 0;
  pub_joints.publish(msg_joints);
  
  ros::spin();
  return 0;
};
