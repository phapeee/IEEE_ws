#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <keyboard/Key.h>

#define M_TO_IN	(39.3700787)
#define R_TO_D	(180 / M_PI)
#define VALID_RANGE	(30 * M_PI / 180)

bool tag_ready_0 = false;
bool tag_ready_1 = false;
bool tag_ready_2 = false;
bool tag_0_cali = false;
bool tag_1_cali = false;
bool tag_2_cali = false;
bool calibrate = false;
tf2::Quaternion tag_0_inv;
tf2::Quaternion tag_1_inv;
tf2::Quaternion tag_2_inv;

void kbCallback(const keyboard::Key::ConstPtr& msg){
	if (msg->code == keyboard::Key::KEY_RETURN) calibrate = true;
}

void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
	static tf2_ros::TransformBroadcaster br;

	if (msg->detections.empty()) {
        	return;
    	}

	geometry_msgs::TransformStamped tran;
	geometry_msgs::Pose tag_pose = msg->detections[0].pose.pose.pose;

	int id = msg->detections[0].id[0];
	tf2::Quaternion q;
	tf2::fromMsg(tag_pose.orientation, q);

	tran.transform.translation.x = tag_pose.position.x * M_TO_IN;
	tran.transform.translation.y = tag_pose.position.z * M_TO_IN;
	tran.transform.translation.z = 0;

	double ang = abs(atan2(tag_pose.position.x, tag_pose.position.z));
//	ROS_INFO("ang: %.3f", ang * R_TO_D);
	if (ang > VALID_RANGE) return;

	tran.header.stamp = ros::Time::now();
	tran.header.frame_id = "camera";
	switch (id){
		case 0:
			if (calibrate){
				ROS_INFO("Tag 0 calibrated!");
				tag_0_inv = q.inverse();
				tag_0_cali = true;
				calibrate = false;
				return;
			}
			if (!tag_0_cali){
				ROS_WARN("Tag 0 have not calibrated!");
				return;
			}
			tran.child_frame_id = "tag_0";
			q *= tag_0_inv;
			tag_ready_0 = true;
			break;
		case 1:
			if (calibrate){
				ROS_INFO("Tag 1 calibrated!");
				tag_1_inv = q.inverse();
				tag_1_cali = true;
				calibrate = false;
				return;
			}
			if (!tag_1_cali){
				ROS_WARN("Tag 1 have not calibrated!");
				return;
			}
			tran.child_frame_id = "tag_1";
			q *= tag_1_inv;
			tag_ready_1 = true;
			break;
		case 2:
			if (calibrate){
				ROS_INFO("Tag 2 calibrated!");
				tag_2_inv = q.inverse();
				tag_2_cali = true;
				calibrate = false;
				return;
			}
			if (!tag_2_cali){
				ROS_WARN("Tag 2 have not calibrated!");
				return;
			}
			tran.child_frame_id = "tag_2";
			q *= tag_2_inv;
			tag_ready_2 = true;
			break;
		default: break;
	}

	q.setX(0);
	q.setZ(q.y());
	q.setY(0);
	q.normalize();
//			ROS_INFO("Rot: x:%.3f, y:%.3f, z:%.3f, w:%.3f", q.x(), q.y(), q.z(), q.w());

	tran.transform.rotation.x = 0;
	tran.transform.rotation.y = 0;
	tran.transform.rotation.z = -q.z();
	tran.transform.rotation.w = q.w();

	br.sendTransform(tran);
}

void broadcastStaticTransform(
    const std::string& parent_frame, const std::string& child_frame,
    const geometry_msgs::Transform& transform_msg) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped static_transform;
    static_transform.header.stamp = ros::Time::now();
    static_transform.header.frame_id = parent_frame;
    static_transform.child_frame_id = child_frame;
    static_transform.transform = transform_msg;

    static_broadcaster.sendTransform(static_transform);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_transform_and_pose_calculation");
    ros::NodeHandle nh;
    ros::Subscriber kb_sub = nh.subscribe("keyboard/keydown", 1, kbCallback);
    ros::Subscriber tag_sub = nh.subscribe("tag_detections", 10, tagCallback);
    ros::Publisher vo_pub = nh.advertise<nav_msgs::Odometry>("vo", 10);

    // Step 1: Broadcast the static transforms
    geometry_msgs::Transform transform_base_to_camera;
    transform_base_to_camera.translation.x = 0.871;  // Example values
    transform_base_to_camera.translation.y = 1.345;
    transform_base_to_camera.translation.z = 0.0;
    tf2::Quaternion q_base_to_camera;
//    q_base_to_camera.setRPY(-M_PI / 2, 0, -M_PI / 2);  // No rotation
    q_base_to_camera.setRPY(0, 0, 0);  // No rotation
    transform_base_to_camera.rotation = tf2::toMsg(q_base_to_camera);

    geometry_msgs::Transform transform_pad_to_world;
    tf2::Quaternion q_pad_to_world;
    transform_pad_to_world.translation.x = -11/3;  // Example values
    transform_pad_to_world.translation.y = -22.5;
    transform_pad_to_world.translation.z = 0;
    q_pad_to_world.setRPY(0, 0, M_PI);  // No rotation
    transform_pad_to_world.rotation = tf2::toMsg(q_pad_to_world);
    broadcastStaticTransform("tag_0", "world_map_0", transform_pad_to_world);

    transform_pad_to_world.translation.x = -31.252;  // Example values
    transform_pad_to_world.translation.y = -45;
    transform_pad_to_world.translation.z = 0;
    q_pad_to_world.setRPY(0, 0, -M_PI / 2);  // No rotation
    transform_pad_to_world.rotation = tf2::toMsg(q_pad_to_world);
    broadcastStaticTransform("tag_1", "world_map_1", transform_pad_to_world);

    transform_pad_to_world.translation.x = -44.5;  // Example values
    transform_pad_to_world.translation.y = -1/8;
    transform_pad_to_world.translation.z = 0;
    q_pad_to_world.setRPY(0, 0, 0);  // No rotation
    transform_pad_to_world.rotation = tf2::toMsg(q_pad_to_world);
    broadcastStaticTransform("tag_2", "world_map_2", transform_pad_to_world);

    geometry_msgs::Transform transform_base_to_world;
    transform_base_to_world.translation.x = 0.0;
    transform_base_to_world.translation.y = 0.0;
    transform_base_to_world.translation.z = 0.0;
    tf2::Quaternion q_base_to_world;
    q_base_to_world.setRPY(0, 0, M_PI/2);  // No rotation
    transform_base_to_world.rotation = tf2::toMsg(q_base_to_world);

    // Broadcast static transforms
    broadcastStaticTransform("base_link", "camera", transform_base_to_camera);
   // broadcastStaticTransform("world_map", "base_link", transform_base_to_world);

    // Step 2: Listen for transforms
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Duration(3.0).sleep();  // Allow time for tf tree to populate
    ros::Rate rate(20);
    tf2::Quaternion inv_q = tf2::Quaternion(0.5, -0.5, 0.5, 0.5).inverse();
    double roll, pitch, yaw;

    nav_msgs::Odometry vo;
    vo.header.frame_id = "vo_link";

        double val;
        for (int i=0; i<36; i++){
                val = 0;
                if (i == 0 || i == 7 || i == 14 || i == 21 || i == 28 || i == 35) val = 0.1;
                vo.pose.covariance[i] = val;
        }


    while(ros::ok()){
	while (tag_ready_0 || tag_ready_1 || tag_ready_2){
	    	try {
			geometry_msgs::TransformStamped transformStamped;
			if (tag_ready_0){
				transformStamped = tfBuffer.lookupTransform("world_map_0", "base_link", ros::Time(0), ros::Duration(3.0));
				tag_ready_0 = false;
			}
			else if (tag_ready_1){
				transformStamped = tfBuffer.lookupTransform("world_map_1", "base_link", ros::Time(0), ros::Duration(3.0));
				tag_ready_1 = false;
			}
			else if (tag_ready_2){
				transformStamped = tfBuffer.lookupTransform("world_map_2", "base_link", ros::Time(0), ros::Duration(3.0));
				tag_ready_2 = false;
			}
			tf2::Quaternion q;
			tf2::fromMsg(transformStamped.transform.rotation, q);
			tf2::Quaternion corrected_q = q * inv_q;
			corrected_q.normalize();
    			//tf2::Matrix3x3 rotation_matrix(corrected_q);
			//rotation_matrix.getRPY(roll, pitch, yaw);

			vo.header.stamp = ros::Time::now();
			vo.pose.pose.position.x = transformStamped.transform.translation.x;
			vo.pose.pose.position.y = transformStamped.transform.translation.y;
			vo.pose.pose.position.z = 0;
			vo.pose.pose.orientation.x = 0;
			vo.pose.pose.orientation.y = 0;
			vo.pose.pose.orientation.z = transformStamped.transform.rotation.z;
			vo.pose.pose.orientation.w = transformStamped.transform.rotation.w;

			vo_pub.publish(vo);
        		// Output the pose of base_link relative to world_map
	        	ROS_INFO("Tra: x:%.3f, y:%.3f", transformStamped.transform.translation.y, -transformStamped.transform.translation.x);
//			ROS_INFO("Rot: x:%.3f, y:%.3f, z:%.3f, w:%.3f", corrected_q.x(), corrected_q.y(), corrected_q.z(), corrected_q.w());
//			ROS_INFO("Rot: R=%.3f, P=%.3f, Y=%.3f", roll * R_TO_D, pitch * R_TO_D, yaw * R_TO_D);
	    	} catch (tf2::TransformException& ex) {
        		ROS_WARN("Transform failed: %s", ex.what());
	    	}
	}
	ros::spinOnce();
	rate.sleep();
    }
    return 0;
}
