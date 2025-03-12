#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sstream>

serial::Serial serial_port;

bool scanUSB(std::string usb_name){
        bool isAlreadyOpen = true;
        ROS_INFO("Finding %s", usb_name.c_str());
        for (int i=0; i<3; i++){
                try {
                        std::string port = "/dev/ttyUSB" + std::to_string(i);
                        serial_port.setPort(port);
                        if (!serial_port.isOpen()) {
                                serial_port.open();
                                isAlreadyOpen = false;
                        }
                } catch (serial::IOException& e) {
                        continue;
                }
                //if (i == 2) return true;
                serial_port.flush();
                ros::Duration(1).sleep();
                serial_port.write("$?\n");
                ros::Duration(0.5).sleep();
                if (serial_port.available()) {
                        std::string device_name = serial_port.readline(65536, "\n");
                        device_name.pop_back();
                        if (device_name == usb_name){
                                ROS_INFO("%s found!", usb_name.c_str());
                                return true;
                        }
                        if (!isAlreadyOpen) serial_port.close();
                        isAlreadyOpen = true;
                }
        }
        return false;
}

// Function to send data to the device
void writeToSerial(const std::string& message) {
    if (serial_port.isOpen()) {
        serial_port.write(message + "\n");  // Send data with newline character
    } else {
        ROS_WARN("Serial port not open.");
    }
}

// Function to parse sensor data
bool parseSensorData(const std::string& data, std::vector<uint16_t>& distances) {
    distances.clear();
    std::istringstream stream(data);
    std::string segment;
    while (std::getline(stream, segment, ' ')) {
        size_t colon_pos = segment.find(":");
        if (colon_pos != std::string::npos) {
            int sensor_id = std::stoi(segment.substr(0, colon_pos));
            uint16_t distance = static_cast<uint16_t>(std::stoi(segment.substr(colon_pos + 1)));
            if (sensor_id >= 0 && sensor_id < 3) {
                if (distances.size() <= sensor_id) {
                    distances.resize(sensor_id + 1);
                }
                distances[sensor_id] = distance;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
    return distances.size() == 3;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tof_sensors_node");
    ros::NodeHandle nh("~");  // Private NodeHandle for parameters

        serial_port.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(timeout);

    if (!scanUSB("tof")){
        ROS_ERROR("Cannot find ToF sensors!");
        return 0;
    }

    // Send initial command to STM32
    writeToSerial("$$");
    ROS_INFO("Sent initialization command to STM32.");

    // Publisher for ToF sensor data
    ros::Publisher tof_pub = nh.advertise<std_msgs::UInt16MultiArray>("/tof_distances", 1000);

    ros::Rate loop_rate(10); // 10 Hz loop rate
    while (ros::ok()) {
        if (serial_port.available()) {
            std::string raw_data = serial_port.readline(65536, "\n");
	    ROS_INFO("%s", raw_data.c_str());
            std::vector<uint16_t> distances;
            if (parseSensorData(raw_data, distances)) {
                std_msgs::UInt16MultiArray sensor_msg;
                sensor_msg.data = distances;
                tof_pub.publish(sensor_msg);
            } else {
                ROS_WARN_STREAM("Invalid sensor data received: " << raw_data);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    serial_port.close();
    return 0;
}
