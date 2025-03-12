#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "april_tag_detection/GetAprilTag.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>
#include <cstdlib>
#include <vector>
#include <fcntl.h>      // For non-blocking socket
#include <sys/select.h> // For select()

#define PORT 8080       // Listening port
#define MAX_RETRIES 5   // Number of retries if data isn't received
#define TIMEOUT_SEC 2   // Timeout in seconds for receiving data
#define ANGLE_THRESHOLD 0.01
#define PACKET_SIZE (8 * sizeof(double)) // 8 double values per packet
#define BUFFER_SIZE 1024

geometry_msgs::Pose tag_pose;
nav_msgs::Odometry pose_est;
char buffer[BUFFER_SIZE];
int client_socket;
//ros::Publisher tag_pub;
geometry_msgs::TransformStamped tag_transform;

bool tag_calibrated[] = {false, false, false, false, false, false, false, false};
bool tag_ready[] = {false, false, false, false, false, false, false, false};
bool isAlign = false;
bool detecting_tag;
bool service_sending;
tf2::Quaternion tag_correction[8];

double yaw_angle;
bool angle_set = false;

struct tag_data{
	uint8_t tag_id;
	uint8_t cam_id;
	geometry_msgs::Pose pose;
};
std::vector<tag_data> tag_poses;

void broadCastPose(const double pose_data[8])
{
	static tf2_ros::TransformBroadcaster broadcaster;
	uint8_t tag_id = pose_data[1];
	uint8_t cam_id = pose_data[0];
	if (abs(tag_id) > 7) return;

	tf2::Quaternion q(pose_data[4], pose_data[5], pose_data[6], pose_data[7]);

	if (!tag_calibrated[tag_id]){
		tag_correction[tag_id] *= q.inverse();
		//ROS_INFO("%.4f,%.4f,%.4f,%.4f", pose_data[4], pose_data[5], pose_data[6], pose_data[7]);
		tag_calibrated[tag_id] = true;
	}

	// correction tag
	q *= tag_correction[tag_id];
	q.normalize();

    // Assign values to the new Pose
	tag_transform.header.stamp = ros::Time::now();
	tag_transform.child_frame_id = "tag_" + std::to_string(tag_id);
	tag_transform.header.frame_id = "cam_" + std::to_string(cam_id);

    tag_transform.transform.translation.x = pose_data[2];
    tag_transform.transform.translation.y = 0;
    tag_transform.transform.translation.z = pose_data[3];

    tag_transform.transform.rotation.x = q.x();
    tag_transform.transform.rotation.y = q.y();
    tag_transform.transform.rotation.z = q.z();
    tag_transform.transform.rotation.w = q.w();

	tag_data tag;
	tag.tag_id = tag_id;
	tag.cam_id = cam_id;
	tag.pose.position.x = pose_data[2];
	tag.pose.position.y = 0;
	tag.pose.position.z = pose_data[3];
	tag.pose.orientation.x = q.x();
	tag.pose.orientation.y = q.y();
	tag.pose.orientation.z = q.z();
	tag.pose.orientation.w = q.w();
	tag_poses.push_back(tag);

	if (tag_id > 4){ // for beacon placer test only. Remove this line after test
		broadcaster.sendTransform(tag_transform);
		tag_ready[tag_id] = true;
	}
}

// Function to request data from the client
bool request_data(int client_socket) {

/*    int retries = 0;
	fd_set read_fds;
    std::string request = "REQ";
    struct timeval timeout;

//    while (retries < MAX_RETRIES) {
	    // Request data from client
	    FD_ZERO(&read_fds);
	    FD_SET(client_socket, &read_fds);
	    send(client_socket, request.c_str(), request.size(), 0);

        // Set socket timeout for receiving data
        timeout.tv_sec = TIMEOUT_SEC;
        timeout.tv_usec = 0;
        //setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof timeout);
		int activity = select(client_socket + 1, &read_fds, NULL, NULL, &timeout);

		if (activity == -1) {
            perror("Select error");
            return false;
        } else if (activity == 0) {
            std::cout << "No tags received!" << std::endl;
            return false;
        } else {
            // Data available, receive it
	        char check_buffer[4] = {0};
	        int bytes_received = recv(client_socket, check_buffer, sizeof(check_buffer) - 1, MSG_PEEK); // Peek at incoming data
            if (bytes_received <= 0) {
                std::cout << "Raspberry Pi 5 disconnected or error occurred." << std::endl;
                return false;
            }

            check_buffer[bytes_received] = '\0'; // Null-terminate for safety

            if (strncmp(check_buffer, "N/A", 3) == 0) {
                // Consume the "N/A" message
                recv(client_socket, check_buffer, sizeof(check_buffer) - 1, 0);
                return false;
            }
			// received tag's data
			else {
                bytes_received = recv(client_socket, received_data, sizeof(double) * 8, 0);
                if (bytes_received == sizeof(double) * 8) {
					broadCastPose(received_data);
					ROS_INFO("CAM: %d, ID: %d",(uint8_t) received_data[0],(uint8_t) received_data[1]);
                }
            }

            // END receiveing tags
            if (strstr(received_data, "END") != nullptr) {
                return true;
            }
        }


    //    retries++;
  //  }
*/
	tag_poses.clear();
	std::string request = "REQ";
	send(client_socket, request.c_str(), request.size(), 0);
	while (true) {
        // Set up file descriptor set for select()
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(client_socket, &read_fds);

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = TIMEOUT_SEC;
        timeout.tv_usec = 0;

        // Wait for data using select()
        int activity = select(client_socket + 1, &read_fds, NULL, NULL, &timeout);

        if (activity == -1) {
            perror("Select error");
            break;
        } else if (activity == 0) {
            std::cout << "Timeout reached. No data received." << std::endl;
            break;
        }

        // Data is available, receive it
        int bytes_received = recv(client_socket, buffer, BUFFER_SIZE - 1, 0);
        if (bytes_received <= 0) {
            std::cout << "Server disconnected or error occurred." << std::endl;
            break;
        }

        buffer[bytes_received] = '\0'; // Null-terminate for string safety

        // Check if "END" is received
        if (std::string(buffer).find("END") != std::string::npos) {
            std::cout << "END detected. Stopping reception." << std::endl;
            return true;
        }

        // Process received data in 8-double packets
        for (int i = 0; i < bytes_received; i += PACKET_SIZE) {
            if (i + PACKET_SIZE <= bytes_received) {
                double received_data[8];
                std::memcpy(received_data, buffer + i, PACKET_SIZE);
				broadCastPose(received_data);
				//ROS_INFO("CAM: %d, ID: %d",(uint8_t) received_data[0],(uint8_t) received_data[1]);
            }
        }
    }

    return false;  // Indicate failure
}

bool getAprilTagPose(april_tag_detection::GetAprilTag::Request &req, 
                     april_tag_detection::GetAprilTag::Response &res) {
	ros::Time timer = ros::Time::now();
	while(detecting_tag){
		if((ros::Time::now() - timer).toSec() >= 3.0) return false;
	}

	unsigned int tag_size = tag_poses.size();
	if (tag_size == 0) return false;

	service_sending = true;

	std::vector<tag_data> tag_to_send;
	int req_tag_id = -1;
	int req_cam_id = -1;
	if (req.tag_id != "*") req_tag_id = std::stoi(req.tag_id);
	if (req.cam_id != "*") req_cam_id = std::stoi(req.cam_id);
	for (int i=0; i<tag_size; i++){
		if (req_tag_id != -1 && req_tag_id != tag_poses[i].tag_id) continue;
		if (req_cam_id != -1 && req_cam_id != tag_poses[i].tag_id) continue;
		tag_to_send.push_back(tag_poses[i]);
	}

	unsigned int tag_size_to_send = tag_to_send.size();

	res.tag_id.resize(tag_size_to_send);
	res.cam_id.resize(tag_size_to_send);
	res.poses.resize(tag_size_to_send);

	for (int i=0; i<tag_size_to_send; i++){
		res.tag_id[i] = tag_to_send[i].tag_id;
		res.cam_id[i] = tag_to_send[i].cam_id;
		res.poses[i] = tag_to_send[i].pose;
	}

	service_sending = false;
    return true;
}

void IMUCallback(const sensor_msgs::Imu::ConstPtr& imu_data){
	tf2::Quaternion imu_q(imu_data->orientation.x, imu_data->orientation.y, imu_data->orientation.z, imu_data->orientation.w);

    // Convert to rotation matrix
    tf2::Matrix3x3 m(imu_q);

    // Extract roll, pitch, yaw
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_angle);
	//ROS_INFO("IMU yaw: %.3f", yaw_angle);
	angle_set = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "capture_apriltag");
    ros::NodeHandle nh;
    ros::Rate rate(20);

	// Subscriber
	ros::Subscriber imu_sub = nh.subscribe("imu_data", 10, IMUCallback);

	// Service
    ros::ServiceServer service = nh.advertiseService("get_april_tag", getAprilTagPose);

	// Publisher
	ros::Publisher tag_pose_pub = nh.advertise<nav_msgs::Odometry>("vo", 10);
    //tag_pub = nh.advertise<geometry_msgs::Pose>("tag_pose", 1);

	// Start async spinner with 2 threads
    ros::AsyncSpinner spinner(2);
    spinner.start(); // Background callback processing

	// cameras' static position
	double camPos_x[] = {5.803, -5.803, 0};
	double camPos_y[] = {1.181, 1.181, 5.803};
	double camRot_Y[] = {-M_PI_2, M_PI_2, 0};

	// broadcast cameras' static frame
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "base_link_tag";
	tf2::Quaternion q;

	for (int i=0; i<3; i++){
		transform.child_frame_id = "cam_" + std::to_string(i);
		transform.transform.translation.x = camPos_x[i];
		transform.transform.translation.y = camPos_y[i];
		transform.transform.translation.z = 0;

		q.setRPY(-M_PI_2, 0, camRot_Y[i]);
		transform.transform.rotation.x = q.x();
		transform.transform.rotation.y = q.y();
		transform.transform.rotation.z = q.z();
		transform.transform.rotation.w = q.w();

		static_broadcaster.sendTransform(transform);
	}

	// AprilTags' static position
	double tagPos_x[] = {-(20.160433 + 2.339567), -(30 + 1.96850393), (41.606299 + 1.96850393), -92.921260};
	double tagPos_z[] = {-0.338583, 44.921260, -0.078740, 20.531496 + 1.968504};
	double tagRot_Y[] = {-M_PI_2, 0, M_PI, -M_PI_2};

	// Broadcast AprilTags' static frame
	for (int i=0; i<8; i++){
		transform.header.frame_id = "tag_" + std::to_string(i);
		transform.child_frame_id = "world_map_" + std::to_string(i);
		transform.transform.translation.x = tagPos_x[i < 5 ? 0 : i - 4];
		transform.transform.translation.y = 0;
		transform.transform.translation.z = tagPos_z[i < 5 ? 0 : i - 4];

		q.setRPY(-M_PI_2, tagRot_Y[i < 5 ? 0 : i - 4], 0);
		transform.transform.rotation.x = q.x();
		transform.transform.rotation.y = q.y();
		transform.transform.rotation.z = q.z();
		transform.transform.rotation.w = q.w();

		static_broadcaster.sendTransform(transform);

		q.setRPY(M_PI, 0, 0);
		tag_correction[i] = q;
	}

	// Creating TCP protocol
    int server_fd;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Socket failed");
        exit(EXIT_FAILURE);
    }

    // Enable SO_REUSEADDR to reuse the port immediately after restarting
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt failed");
        exit(EXIT_FAILURE);
    }

    // Bind socket to IP and port
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    // Listen for connections
    if (listen(server_fd, 3) < 0) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << PORT << "...\n";

    // Accept client connection
    if ((client_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
	perror("Accept failed");
	exit(EXIT_FAILURE);
    }

    std::cout << "Client connected.\n";

	pose_est.pose.pose.position.z = 0;
	pose_est.pose.pose.orientation.x = 0;
	pose_est.pose.pose.orientation.y = 0;
	pose_est.pose.pose.orientation.z = 0;
	pose_est.pose.pose.orientation.w = 1.0;

	// Initialize covariance (6x6 matrix stored in a 1D array)
    for (int i = 0; i < 36; i++) {
        pose_est.pose.covariance[i] = 0.0;  // Default high uncertainty
    }

    pose_est.pose.covariance[0] = 0.01;  // Variance of X
    pose_est.pose.covariance[7] = 0.01;  // Variance of Y
	pose_est.pose.covariance[14] = 1e6;   // Variance of Z (high uncertainty)
    pose_est.pose.covariance[21] = 1e6;   // Variance of Roll (high uncertainty)
    pose_est.pose.covariance[28] = 1e6;   // Variance of Pitch (high uncertainty)
    pose_est.pose.covariance[35] = 1e6;   // Variance of Yaw (high uncertainty)

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	int tag_count;

	while (ros::ok()){
		while(!service_sending){}
		detecting_tag = true;
		request_data(client_socket);
		detecting_tag = false;
		tag_count = 0;
		pose_est.pose.pose.position.x = 0;
		pose_est.pose.pose.position.y = 0;
		if (angle_set && (std::abs(yaw_angle) < ANGLE_THRESHOLD ||  // Check for 0
	    	std::abs(std::abs(yaw_angle) - M_PI_2) < ANGLE_THRESHOLD ||  // Check for π/2 (90°)
    		std::abs(std::abs(yaw_angle) - M_PI) < ANGLE_THRESHOLD)){
			for (int i=0; i<8; i++){
				if (!tag_ready[i]) continue;
				tag_count++;
				geometry_msgs::TransformStamped transformStamped;
				tag_ready[i] = false;
				std::string world_name = "world_map_" + std::to_string(i);
				try{
					transformStamped = tfBuffer.lookupTransform(world_name, "base_link_tag", ros::Time(0));
				}
				catch (tf2::TransformException &ex){
					ROS_WARN("Could NOT transform base_link_tag to %s: %s", world_name.c_str(), ex.what());
				}
				//ROS_INFO("x: %.2f, y: %.2f", transformStamped.transform.translation.x, transformStamped.transform.translation.y);
				pose_est.pose.pose.position.x += transformStamped.transform.translation.x;
				pose_est.pose.pose.position.y += transformStamped.transform.translation.y;
			}
			pose_est.pose.pose.position.x /= tag_count;
			pose_est.pose.pose.position.y /= tag_count;
			pose_est.header.stamp = ros::Time::now();
			tag_pose_pub.publish(pose_est);
		}
		rate.sleep();
	}

    // Close sockets
    close(client_socket);
    close(server_fd);

    return 0;
}
