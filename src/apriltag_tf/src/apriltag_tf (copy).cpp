#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#define M_TO_IN	(39.3700787)

apriltag_ros::AprilTagDetectionArray tag;
bool tag_ready = false;

void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
/*
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped tran;
	geometry_msgs::Pose tag_pose = msg->detections[0].pose.pose.pose;

	int id = msg->detections[0].id[0];

	tran.header.stamp = ros::Time::now();
	tran.header.frame_id = "camera";
	switch (id){
		case 1:	tran.child_frame_id = "PAD_2"; break;
		default: break;
	}
/*
	tf2::Transform inv_tran =
		tf2::Transform(
			tf2::Quaternion(tag_pose.orientation.x,
					tag_pose.orientation.y,
			  		tag_pose.orientation.z,
			  		tag_pose.orientation.w),
			tf2::Vector3(tag_pose.position.x * M_TO_IN,
				     tag_pose.position.y * M_TO_IN,
				     tag_pose.position.z * M_TO_IN)
		).inverse();

	tf2::Vector3 translation = inv_tran.getOrigin();
	tf2::Quaternion rotation = inv_tran.getRotation();

	tran.transform.translation.x = -translation.z();
	tran.transform.translation.y = -translation.y();
	tran.transform.translation.z = 0;

	tran.transform.rotation = tf2::toMsg(rotation);;

	tran.transform.translation.x = tag_pose.position.x;
	tran.transform.translation.y = 0;
	tran.transform.translation.z = tag_pose.position.z;

	tran.transform.rotation = tag_pose.orientation;


	br.sendTransform(tran);
	tag_ready = true;
	*/

	static tf2_ros::TransformBroadcaster br;

    if (msg->detections.empty()) {
        //ROS_WARN("No tags detected!");
        return;
    }

    // Extract the pose of the detected tag
    geometry_msgs::Pose tag_pose = msg->detections[0].pose.pose.pose;
    int id = msg->detections[0].id[0];

    // Check if the detected tag is the one we're interested in
    if (id == 1) {
        // Broadcast the dynamic transform: camera -> PAD_2
        geometry_msgs::TransformStamped tran;
        tran.header.stamp = ros::Time::now();
        tran.header.frame_id = "camera";
        tran.child_frame_id = "PAD_2";

        tran.transform.translation.x = tag_pose.position.x;
        tran.transform.translation.y = tag_pose.position.y;
        tran.transform.translation.z = tag_pose.position.z;

        tran.transform.rotation = tag_pose.orientation;

        br.sendTransform(tran);
	tag_ready = true;
        //ROS_INFO_STREAM("Broadcasting dynamic transform: camera -> PAD_2");
    }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "arptiltag_tf");
	ros::NodeHandle nh;
	//ros::Subscriber tag_sub = nh.subscribe("tag_detections", 10, tagCallback);

	double tags_x[] = {0};
	double tags_y[] = {0};
	double tags_ang[] = {0};

	tf2_ros::StaticTransformBroadcaster br;
	geometry_msgs::TransformStamped tran;

	tran.header.stamp = ros::Time::now();
/*
	tran.header.frame_id = "base_link";
	tran.child_frame_id = "camera";
	tran.transform.translation.x = 0.871;
	tran.transform.translation.y = 1.345;
	tran.transform.translation.z = 0;

	tf2::Quaternion q;
	q.setRPY(-M_PI/2, 0, -M_PI/2);
	tran.transform.rotation.x = q.x();
	tran.transform.rotation.y = q.y();
	tran.transform.rotation.z = q.z();
	tran.transform.rotation.w = q.w();

	br.sendTransform(tran);

	tran.header.frame_id = "world_map";

	tran.transform.translation.x = 0;
	tran.transform.translation.y = 0;
	tran.transform.translation.z = 0;

	tran.transform.rotation.x = 0;
	tran.transform.rotation.y = 0;
	tran.transform.rotation.z = 0;
	tran.transform.rotation.w = 1;

//	tran.child_frame_id = "base_link";

//	br.sendTransform(tran);

	tran.child_frame_id = "PAD_2";

	tran.transform.translation.x = 17.171;
	tran.transform.translation.y = 8.075;

	br.sendTransform(tran);
*/

// Static Transform: world_map -> PAD_2
tran.header.frame_id = "world_map";
tran.child_frame_id = "PAD_2";
tran.transform.translation.x = 17.171;
tran.transform.translation.y = 8.075;
tran.transform.translation.z = 0;
tran.transform.rotation.x = 0;
tran.transform.rotation.y = 0;
tran.transform.rotation.z = 0;
tran.transform.rotation.w = 1;
br.sendTransform(tran);

// Static Transform: base_link -> camera
tran.header.frame_id = "camera";
tran.child_frame_id = "base_link";
tran.transform.translation.x = 0.871;
tran.transform.translation.y = 1.345;
tran.transform.translation.z = 0;

tf2::Quaternion q;
q.setRPY(-M_PI / 2, 0, -M_PI / 2);
tran.transform.rotation.x = q.x();
tran.transform.rotation.y = q.y();
tran.transform.rotation.z = q.z();
tran.transform.rotation.w = q.w();
br.sendTransform(tran);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
//        ros::Duration(5.0).sleep();
	ros::Rate rate(30);

while (ros::ok()){
	//if (tag_ready){
	tag_ready = false;
    try
    {
        // Wait for the transforms to be available

        // Get the transform from "PAD_2" to "base_link"
        geometry_msgs::TransformStamped T_tag1_base_link = tfBuffer.lookupTransform("base_link", "PAD_2", ros::Time(0));

        // Get the transform from "PAD_2" to "world_map"
        geometry_msgs::TransformStamped T_tag1_world_map = tfBuffer.lookupTransform("world_map", "PAD_2", ros::Time(0));

        // Compute the inverse of T_tag1_base_link
        tf2::Transform tf_tag1_base_link;
        tf2::fromMsg(T_tag1_base_link.transform, tf_tag1_base_link);
        tf2::Transform tf_base_link_tag1 = tf_tag1_base_link.inverse();

        // Compute T_base_link_world_map = T_tag1_world_map * T_base_link_tag1
        tf2::Transform tf_tag1_world_map;
        tf2::fromMsg(T_tag1_world_map.transform, tf_tag1_world_map);

        tf2::Transform tf_base_link_world_map = tf_tag1_world_map * tf_base_link_tag1;

        // Convert to geometry_msgs::TransformStamped for publishing or logging
        geometry_msgs::TransformStamped T_base_link_world_map;
        T_base_link_world_map.header.stamp = ros::Time::now();
        T_base_link_world_map.header.frame_id = "world_map";
        T_base_link_world_map.child_frame_id = "base_link";
        T_base_link_world_map.transform = tf2::toMsg(tf_base_link_world_map);

        ROS_INFO_STREAM("Pose of base_link w.r.t. world_map: " << T_base_link_world_map);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("Transform error: %s", ex.what());
    }
//}
	ros::spinOnce();
	rate.sleep();
}
//	ros::spin();
	return 0;
}
