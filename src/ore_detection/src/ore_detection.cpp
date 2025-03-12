#include <ros/ros.h>
#include <iostream>
#include <string>
#include <cstring>    // For memset
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>   // For close()

#define PORT 1234    // Port number (match this with the server)
#define SERVER_IP "192.168.1.2"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_tcp_client");
    ros::NodeHandle nh;

    int sock = 0;
    struct sockaddr_in server_addr;
    char buffer[5000] = {0};

    // Create socket
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ROS_ERROR("Socket creation error");
        return -1;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    // Convert IPv4 address to binary
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {  // Replace with server's IP
        ROS_ERROR("Invalid address or Address not supported");
        return -1;
    }

    // Connect to server
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ROS_ERROR("Connection failed");
        return -1;
    }

    ROS_INFO("Server connected!");

    fd_set read_fds;
    struct timeval timeout;
    int select_ret;

    while(ros::ok()){
	// Initialize the file descriptor set
        FD_ZERO(&read_fds);
        FD_SET(sock, &read_fds);

        // Set timeout (e.g., 1 second)
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        // Check if there's data available on the socket
        select_ret = select(sock + 1, &read_fds, nullptr, nullptr, &timeout);
        if (select_ret > 0) {
            // Data is available, read it
            if (FD_ISSET(sock, &read_fds)) {
                int valread = read(sock, buffer, sizeof(buffer) - 1);
                if (valread > 0) {
                    buffer[valread] = '\0';  // Null-terminate the string
                    ROS_INFO("%s", buffer);
                } else if (valread == 0) {
                    ROS_WARN("Server closed the connection");
                    break;
                } else {
                    ROS_ERROR("Read error");
                    break;
                }
            }
        } else if (select_ret == 0) {
            // Timeout, no data received
        } else {
            // An error occurred
            ROS_ERROR("select() error");
            break;
        }

	ros::spinOnce();
    }

    // Close the socket
    close(sock);

    return 0;
}
