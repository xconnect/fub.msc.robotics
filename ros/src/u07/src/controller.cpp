#include <ros/ros.h>
#include <tf/tf.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include "controller.h"

PID::PID(double _kp, double _ki, int _memoryi, double _kd):
		kp(_kp),
		ki(_ki),
		memoryi(std::max(1, _memoryi)),
		kd(_kd),
		sum(0.0) {
	memory.push_back(0.0);
}

const double PID::control(double error) {
	const double asc = error - memory.back();
	sum += error;
	memory.push_back(error);

	if(memory.size() > memoryi) {
		sum -= memory.front();
		memory.erase(memory.begin());
	}

	return kp*error + ki*sum + kd*asc;
}

ros::Subscriber subOdo;
ros::Publisher pubY;
ros::Publisher pubError;
ros::Publisher pubPidout;
ros::Publisher pubDrive;
double y     = 0.0;
double error = 0.0;

void odometryCB(const nav_msgs::Odometry &msg) {
	double uyaw = tf::getYaw(msg.pose.pose.orientation);
	double syaw = uyaw <= M_PI ? uyaw : uyaw - 2.0 * M_PI;
	y     = msg.pose.pose.position.y;
	error = Controller::targety == y ? 0
			: atan2(Controller::targety - y, Controller::lookahead) - syaw;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "controller");
	ros::Time::init();
	ros::Rate r(Controller::rate);
	ros::NodeHandle n;
	subOdo    = n.subscribe("/ackermann_vehicle/odom", 100, odometryCB);
	pubY      = n.advertise<std_msgs::Float64>("/ackermann_vehicle/y", 100);
	pubError  = n.advertise<std_msgs::Float64>("/ackermann_vehicle/error", 100);
	pubPidout = n.advertise<std_msgs::Float64>("/ackermann_vehicle/pidout", 100);
	pubDrive  = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd", 100);
	std_msgs::Float64 msg_plot;
	ackermann_msgs::AckermannDriveStamped msg_drive;
	msg_drive.drive.jerk                    = 0;
	msg_drive.drive.steering_angle_velocity = Controller::steer_vel;
	msg_drive.drive.speed                   = Controller::speed;
	msg_drive.drive.acceleration            = Controller::accel;
	PID pid(atof(argv[1]), atof(argv[2]), atoi(argv[3]), atof(argv[4]));
	ros::Duration(5).sleep();

	for(int i=0; i<Controller::iterations; ++i) {
		const double pidout = pid.control(error);
		msg_plot.data = y;
		pubY.publish(msg_plot);
		msg_plot.data = error;
		pubError.publish(msg_plot);
		msg_plot.data = pidout;
		pubPidout.publish(msg_plot);
		msg_drive.header.stamp         = ros::Time(0);
		msg_drive.drive.steering_angle = pidout;
		pubDrive.publish(msg_drive);
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
