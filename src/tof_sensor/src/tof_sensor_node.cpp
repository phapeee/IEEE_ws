#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <serial/serial.h>
#include <sstream>
#include <string>
#include <vector>
#include <regex>
#include "tof_sensor/Tof.h"  // Replace with your actual package name
#include "tof_sensor/kalman.hpp"
#include <cmath>

#define INIT_SAMPLE_SIZE (10)

serial::Serial serial_port;
bool arduino_active = false;
bool sensor_init = false;
double cnt;
double init_val[3];
double covariance[] = {5, 16.80, 5};
std::function<double(double)> cali1 = [](double x) { return 1.0027 * x - 46.663; };
std::function<double(double)> cali2 = [](double x) { return 0.000008*pow(x,3) - 0.0032*pow(x,2) + 1.4789*x - 57.148; };
std::function<double(double)> cali3 = [](double x) { return 0.00003*pow(x,3) - 0.0135*pow(x,2) + 3.0106*x - 127.18; };
std::array<std::function<double(double)>, 3> adjust = {cali1, cali2, cali3};
std::vector<SimpleKalmanFilter> kf;

bool tofServiceCallback(tof_sensor::Tof::Request &req,
                        tof_sensor::Tof::Response &res)
{
    if (!serial_port.isOpen()) {
        ROS_ERROR("Serial port not open.");
        return false;
    }

    std::string command = req.activate ? "$1\n" : "$0\n";
	arduino_active = req.activate;
	sensor_init = !req.activate;
    serial_port.write(command);
	cnt = 0;
	init_val[0]=0;
	init_val[1]=0;
	init_val[2]=0;

    return true;
}

std::vector<double> parseSensorData(const std::string &data) {
    std::vector<double> distances(3, 0);
    std::regex pattern(R"(0:(\d+),1:(\d+),2:(\d+))");
    std::smatch match;

    if (std::regex_search(data, match, pattern) && match.size() == 4) {
        distances[0] = std::stoi(match[1]);
        distances[1] = std::stoi(match[2]);
        distances[2] = std::stoi(match[3]);
    }

    return distances;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tof_sensor");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("tof_sensor", 10);
    ros::ServiceServer service = nh.advertiseService("tof_srv", tofServiceCallback);

    serial_port.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial_port.setTimeout(timeout);
    serial_port.setPort("/dev/USB_TOF");

    try {
        serial_port.open();
    } catch (serial::IOException &e) {
        ROS_ERROR("Unable to open port: %s", e.what());
        return 1;
    }

    if (!serial_port.isOpen()) {
        ROS_ERROR("Cannot open subsystem (Arduino Mega)!");
        return 1;
    }
	serial_port.write("$0\n");
    ros::Rate rate(10); // 10Hz

    while (ros::ok()) {
        ros::spinOnce();

        if (serial_port.available()) {
			if (arduino_active){
	            std::string data = serial_port.readline(1024, "\r\n");
//				std::cout << data << std::endl;
	            size_t start = data.find("0:");
	            if (start != std::string::npos) {
	                std::string sensor_data = data.substr(start); // Skip "@@"
	                auto distances = parseSensorData(sensor_data);

	                std_msgs::Float64MultiArray msg;
	                msg.data = distances;
					if (!sensor_init){
						for (int i=0; i<3; i++){
							init_val[i] += msg.data[i];
						}
						if (cnt++ >= INIT_SAMPLE_SIZE){
							sensor_init = true;
							kf.clear();
							for (int i=0; i<3; i++){
								init_val[i] /= cnt;
								kf.emplace_back(1.0, covariance[i], covariance[i], init_val[i]);
							}
						}
					}
					else {
						for (int i=0; i<3; i++){
							msg.data[i] = adjust[i](kf[i].update(msg.data[i]));
//							msg.data[i] = init_val[i];
//							init_val[i] = 0;
						}
//						cnt = 0;
//						arduino_active = false;
//						sensor_init = false;
						pub.publish(msg);
					}
	            }
			}
            serial_port.flush();
        }

        rate.sleep();
    }

	serial_port.write("$0\n");
    serial_port.close();
    return 0;
}
