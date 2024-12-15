#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

#define WHEEL_RADIUS    (2.95275591 / 2)
#define VD_HD           (8.6975)
#define COEFF_R		(WHEEL_RADIUS/4)
#define COEFF_M		(1/VD_HD)
#define MAX_SPEED	(25)
#define TIME_COEFF	(0.000001)
#define INIT_X		(31.25)
#define INIT_Y		(6)

ros::Publisher wheel_pub;
//double dt = TIME_COEFF;
nav_msgs::Odometry pose_est;

void poseEstimationCallback(const std_msgs::Float64MultiArray& msg){
	static double angle = 0;
	double dt = msg.data[0] * TIME_COEFF;
	double wheel_1 = msg.data[1] * COEFF_R;
	double wheel_2 = msg.data[2] * COEFF_R;
	double wheel_3 = msg.data[4] * COEFF_R;
	double wheel_4 = msg.data[3] * COEFF_R;
	double temp;
	if (abs(wheel_1) > MAX_SPEED || abs(wheel_2) > MAX_SPEED ||
		abs(wheel_3) > MAX_SPEED || abs(wheel_4) > MAX_SPEED) return;
	if (wheel_1 != 0.0 || wheel_2 != 0.0 ||
		wheel_3 != 0.0 || wheel_4 != 0.0){
		pose_est.pose.pose.position.x += (wheel_1 + wheel_2 + wheel_3 + wheel_4) * dt;
		pose_est.pose.pose.position.y += (-wheel_1 + wheel_2 + wheel_3 - wheel_4) * dt;
		angle += (wheel_1 - wheel_2 + wheel_3 - wheel_4) * COEFF_M * dt;
		if (angle > M_PI) angle -= M_PI_2;
		else if (angle < -M_PI) angle += M_PI_2;
		temp = angle / 2;
		pose_est.pose.pose.orientation.w = cos(temp);
		pose_est.pose.pose.orientation.z = sin(temp);
//		ROS_INFO("X: %.3f, Y: %.3f, Ang: %.3f", pose_est.pose.pose.position.x, pose_est.pose.pose.position.y, angle);
//		ROS_INFO("1: %.3f, 2: %.3f, 3: %.3f, 4: %.3f", wheel_1, wheel_2, wheel_3, wheel_4);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "mecanum_odometry");
	ros::NodeHandle nh;
	ros::Rate rate(60);
	wheel_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
	ros::Subscriber wheel_sub = nh.subscribe("wheel_speed", 100, poseEstimationCallback);

	pose_est.header.frame_id = "odom_link";

	pose_est.pose.pose.position.x = INIT_X;
	pose_est.pose.pose.position.y = INIT_Y;
	pose_est.pose.pose.position.z = 0;

	pose_est.pose.pose.orientation.x = 0;
	pose_est.pose.pose.orientation.y = 0;
	pose_est.pose.pose.orientation.z = 0;
	pose_est.pose.pose.orientation.w = 1;

	double val;
	for (int i=0; i<36; i++){
		val = 0;
		if (i == 0 || i == 7 || i == 14 || i == 21 || i == 28 || i == 35) val = 0.1;
		pose_est.pose.covariance[i] = val;
	}

	while (ros::ok()){
		pose_est.header.stamp = ros::Time::now();
		wheel_pub.publish(pose_est);
		ros::spinOnce();
		rate.sleep();
	}
}
