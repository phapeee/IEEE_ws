#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#define WAITING_TIME	(2.5)	// Wait for IMU to be stablized
#define INIT_ANGLE	(M_PI_2)

ros::Publisher imu_pub;
ros::Time timer;
tf2::Quaternion offset_imu;
sensor_msgs::Imu calibrated_imu;

void imuPoseCallback(const sensor_msgs::Imu& msg){
	static bool calibrated = false;
	static bool waiting = false;
	static bool calibrating = false;
	static ros::Time timer;

	if (!calibrated){
		if (!calibrating){
			timer = ros::Time::now();
			calibrating = true;
			return;
		}
		else if ((ros::Time::now() - timer).toSec() < 5) {
			return;
		}
		tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

		// Create a quaternion for initial angle rotation around Z-axis
    	tf2::Quaternion rotation_quat;
    	rotation_quat.setRPY(0, 0, INIT_ANGLE);  // Yaw angle

		offset_imu = rotation_quat * q.inverse();
    	offset_imu.normalize();

		calibrated = true;
		ROS_INFO("IMU calibrated!");
	}

	tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

	q *= offset_imu;
	q.normalize();

	calibrated_imu.header.stamp = ros::Time::now();
	calibrated_imu.orientation.x = q.x();
	calibrated_imu.orientation.y = q.y();
	calibrated_imu.orientation.z = q.z();
	calibrated_imu.orientation.w = q.w();

	calibrated_imu.orientation_covariance = msg.orientation_covariance;
//	double roll, pitch, yaw;
//	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//	ROS_INFO("R: %.3f, P: %.3f, Y:%.3f", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
//	double yaw = tf::getYaw(calibrated_imu.orientation) * 180 / M_PI;
//	ROS_INFO("%.2f", yaw);
	imu_pub.publish(calibrated_imu);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "imu_tf");
	ros::NodeHandle nh;
	imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
	ros::Subscriber imu_sub = nh.subscribe("wit/imu", 10, imuPoseCallback);

	geometry_msgs::TransformStamped transform;
	tf2_ros::StaticTransformBroadcaster br;

	calibrated_imu.header.frame_id = "imu_link";
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "base_link";
	transform.child_frame_id = "imu_link";
	transform.transform.translation.x = 0;
	transform.transform.translation.y = 0;
	transform.transform.translation.z = 0;
	transform.transform.rotation.x = 0;
	transform.transform.rotation.y = 0;
	transform.transform.rotation.z = 0;
	transform.transform.rotation.w = 1;
	br.sendTransform(transform);

	ros::spin();
	return 0;
}
