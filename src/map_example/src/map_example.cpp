#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <mining_map/Map.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <wiringPi.h>

//map::Map ieee_map(0);
map::Map ieee_caution(1);


void botCallBack(const geometry_msgs::Pose& pose){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0));
        transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,  pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_map", "sensor"));
}

void updateBotCallback(const geometry_msgs::Pose &pose){
//	ieee_caution.updateBot(pose);
}

void dest_action_0(bool& started, bool& done){
	static ros::Time timer;
	if (!started && !done){
		started = true;
		timer = ros::Time::now();
	}
	ros::Duration count_down = ros::Time::now() - timer;
	if (count_down.toSec() >= 5.0) {
		done = true;
	}
}

void start_bot(bool& started, bool& done){
	static ros::Time timer;
	if (!started && !done){
		started = true;
	}
	if (digitalRead(27) == LOW) {
		done = true;
		digitalWrite(17, HIGH);
	}
}

void dest_action_wait(bool& started, bool& done){
	static ros::Time timer;
	if(!started && !done){
		started = true;
		timer = ros::Time::now();
	}
	ros::Duration count_down = ros::Time::now() - timer;
	if (count_down.toSec() >= 1.0){
		done = true;
	}
}

void reset(){
	digitalWrite(17, LOW);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "IEEE_map");
	ros::NodeHandle nh;
	ros::Rate rate(30);

	ros::Subscriber bot_sub = nh.subscribe("IMU_sensor", 10, updateBotCallback);

	//tf::TransformListener listener;
	//tf::StampedTransform sensor_transform;

//	ieee_map.init(&nh, "0", false);
	ieee_caution.init(&nh, "1", true);

	geometry_msgs::Vector3 box_size;
	box_size.x = 4;
	box_size.y = 4;
	box_size.z = 1;
	ieee_caution.createBotController(box_size, 6);

	box_size.x = 2.5;
	box_size.y = 2.5;
	ieee_caution.createContainersController(box_size, 3.5);

	unsigned int pad_num = 0;
	geometry_msgs::Pose bot_pose = ieee_caution.original_bot_pose;
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(bot_pose.position.x, bot_pose.position.y, 0), 0, map::Map::doNothing, start_bot);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(bot_pose.position.x, 9, 0), 0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(9, 9, 0), M_PI / 2.0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(9, 36, 0), 0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(15, 36, 0), -M_PI / 2.0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(15, 22.5, 0), 0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(83, 22.5, 0), 0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(83, 22.5, 0), M_PI / 2.0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(83, 36, 0), M_PI / 2.0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(83, 36, 0), -M_PI / 2.0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(83, 9, 0), -M_PI / 2.0, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(61.75 + 9, 22.5, 0), M_PI, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(55.25 - 9, 22.5, 0), M_PI, map::Map::doNothing, dest_action_wait);
	ieee_caution.addCheckPoint(pad_num++, tf::Vector3(bot_pose.position.x, bot_pose.position.y, 0), M_PI / 2, map::Map::doNothing, dest_action_wait);
	ieee_caution.publishField();

	ieee_caution.reset = reset;

	wiringPiSetupGpio();
	pinMode(17, OUTPUT);
	pinMode(27, INPUT);

	while(ros::ok()){

		ieee_caution.checkCollision();
		ieee_caution.followPath();
//		ieee_caution.pseudoMoveBot();
		ieee_caution.loop();
/*
		ros::Time now = ros::Time::now();

		try{
			listener.waitForTransform("/bot", "/sensor", now, ros::Duration(1.0));
			listener.lookupTranform("/bot", "/sensor", now, transform);
		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
*/
		ros::spinOnce();
		rate.sleep();
	}
}
