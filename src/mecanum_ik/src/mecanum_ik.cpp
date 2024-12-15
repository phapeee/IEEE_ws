#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <math.h>

#define WHEEL_RADIUS	(2.95275591 / 2)
#define VD_HD		(8.7)
#define SCALE		(50)

std_msgs::Float64MultiArray wheel_ang_vel;
ros::Publisher wheel_vel_pub;

// calculate the local speed of bot with given 3D vector velocity in which x and y components are desired local linear velocities and z component desried rotaiional velocity
//     | 1 -1 -(l+w) | |x|
// 1/r*| 1  1  (l+w) |*|y|
//     | 1 -1  (l+w) | |z|
//     | 1  1 -(l+w) |
// return value is a 4D vector in which x is top left, y is top right, z is bottom right, w is bottom left
void inverse_kinematic_callback(const geometry_msgs::Vector3 &bot_vel)
{
	double cos_phi = cos(bot_vel.z);
	double sin_phi = sin(bot_vel.z);

	double local_vel_x = cos_phi * bot_vel.x + sin_phi * bot_vel.y;
	double local_vel_y = cos_phi * bot_vel.y - sin_phi * bot_vel.x;

	double mul_z = VD_HD * bot_vel.z / WHEEL_RADIUS;
	double div_x = local_vel_x / WHEEL_RADIUS;
	double div_y = local_vel_y / WHEEL_RADIUS;
	wheel_ang_vel.data[0] = round((div_x - div_y - mul_z) * SCALE);
	wheel_ang_vel.data[1] = round((div_x + div_y + mul_z) * SCALE);
	wheel_ang_vel.data[2] = round((div_x - div_y + mul_z) * SCALE);
	wheel_ang_vel.data[3] = round((div_x + div_y - mul_z) * SCALE);
	wheel_vel_pub.publish(wheel_ang_vel);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "mecanum_ik");
	//ros::Rate rate(60);
	ros::NodeHandle nh;
	ros::Subscriber bot_vel_sub = nh.subscribe("Bot_Velocities", 1, inverse_kinematic_callback);
	wheel_vel_pub = nh.advertise<std_msgs::Float64MultiArray>("wheel_ang_vels", 1);

	wheel_ang_vel.data = {0,0,0,0};

	ros::spin();
	return 1;
}
