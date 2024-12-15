#include <ros/ros.h>
#include <keyboard/Key.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>

#define INCREMENT	(0.1)
#define DEOUNCE_TIME	(0.5)

geometry_msgs::Vector3 velocities;
ros::Publisher bot_vel_pub;
ros::Time timer;
double max_vel;
double max_ang_vel;
using namespace keyboard;
bool running = false;

void keydownCallback(const keyboard::Key &key_msg){
	if (running) return;
	running = true;
	timer = ros::Time::now();
	bool changed = false;
	switch (key_msg.code){
		case Key::KEY_UP:
			velocities.x = max_vel;
			changed = true;
			break;
		case Key::KEY_DOWN:
			velocities.x = -max_vel;
			changed = true;
			break;
		case Key::KEY_LEFT:
			velocities.y = -max_vel;
			changed = true;
			break;
		case Key::KEY_RIGHT:
			velocities.y = max_vel;
			changed = true;
			break;
		case Key::KEY_q:
			velocities.z = max_ang_vel;
			changed = true;
			break;
		case Key::KEY_w:
			velocities.z = -max_ang_vel;
			changed = true;
			break;
		case Key::KEY_i:
			max_vel += INCREMENT;
			ROS_INFO("Max linear velocity: %.1f", max_vel);
			break;
		case Key::KEY_k:
			max_vel -= INCREMENT;
			ROS_INFO("Max linear velocity: %.1f", max_vel);
			break;
		case Key::KEY_j:
			max_ang_vel -= INCREMENT;
			ROS_INFO("Max angular velocity: %.1f", max_ang_vel);
			break;
		case Key::KEY_l:
			max_ang_vel += INCREMENT;
			ROS_INFO("Max angular velocity: %.1f", max_ang_vel);
			break;
		case Key::KEY_s:
			velocities.x = 0;
			velocities.y = 0;
			velocities.z = 0;
			changed = true;
			break;
		default: break;
	}
	if (changed) bot_vel_pub.publish(velocities);
}

void keyupCallback(const keyboard::Key &key_msg){
//	ROS_INFO("Key code %d, i=%d", key_msg.code, Key::KEY_i);
	if (running) return;
	bool changed = false;
	switch (key_msg.code){
		case Key::KEY_UP:
			velocities.x -= max_vel;
			velocities.x = 0;
			changed = true;
			break;
		case Key::KEY_DOWN:
			velocities.x = 0;
			changed = true;
			break;
		case Key::KEY_LEFT:
			velocities.y = 0;
			changed = true;
			break;
		case Key::KEY_RIGHT:
			velocities.y = 0;
			changed = true;
			break;
		case Key::KEY_q:
			velocities.z = 0;
			changed = true;
			break;
		case Key::KEY_w:
			velocities.z = 0;
			changed = true;
			break;
		default: break;
	}
	if (changed) bot_vel_pub.publish(velocities);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "keyboard_control_node");
	ros::NodeHandle nh;
	bot_vel_pub = nh.advertise<geometry_msgs::Vector3>("Bot_Velocities", 1);
	ros::Subscriber keydown_sub = nh.subscribe("keyboard/keydown", 1, keydownCallback);
	ros::Subscriber keyup_sub = nh.subscribe("keyboard/keyup", 1, keyupCallback);

	max_vel = 2.5;
	max_ang_vel = 0.25;

	while (ros::ok()){
		if (running && (ros::Time::now() - timer).toSec() >= 1) {
			velocities.x = 0;
			velocities.y = 0;
			velocities.z = 0;
			running = false;
			bot_vel_pub.publish(velocities);
		}
		ros::spinOnce();
	}
	//ros::spin();
	return 1;
}
