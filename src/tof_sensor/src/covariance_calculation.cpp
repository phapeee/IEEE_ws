#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <vector>
#include <numeric>
#include <cmath>

#define SAMPLE_SIZE (1000)

std::vector<double> dist_0_log, dist_1_log, dist_2_log;

double calculateVariance(const std::vector<double> &data) {
    if (data.size() < 2) return 0.0;

    double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

    double sum_sq_diff = 0.0;
    for (const auto &value : data) {
        sum_sq_diff += (value - mean) * (value - mean);
    }

    return sum_sq_diff / (data.size() - 1); // Sample variance
}

void callback(const std_msgs::UInt32MultiArray::ConstPtr& msg){
	if (dist_0_log.size() < SAMPLE_SIZE){
		dist_0_log.push_back(msg->data[0]);
		dist_1_log.push_back(msg->data[1]);
		dist_2_log.push_back(msg->data[2]);
	}
		ROS_INFO("%d, %d, %d", msg->data[0], msg->data[1], msg->data[2]);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "cova_calc");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tof_sensor", 10, callback);

	while(ros::ok() && dist_0_log.size() < SAMPLE_SIZE){ros::spinOnce();}

    double var0 = calculateVariance(dist_0_log);
    double var1 = calculateVariance(dist_1_log);
    double var2 = calculateVariance(dist_2_log);

    ROS_INFO("Variance (Sensor 0, 1, 2): %.2f, %.2f, %.2f", var0, var1, var2);

    // Optionally clear and re-log
    dist_0_log.clear();
    dist_1_log.clear();
    dist_2_log.clear();

	return 1;
}
