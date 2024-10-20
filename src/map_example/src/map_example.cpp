#include <ros/ros.h>
#include <mining_map/Map.h>

map::Map ieee_map(0);
map::Map ieee_caution(1);

int main(int argc, char** argv){
	ros::init(argc, argv, "map_view");
	ros::NodeHandle nh;
	ros::Rate rate(30);

	ieee_map.init_visualization(&nh, "0");
	ieee_caution.init_visualization(&nh, "1");

	ieee_caution.setFieldWidth(0.25);
	ieee_caution.setContainersWidth(0.25);
	ieee_caution.setBotWidth(0.25);
	ieee_map.setFieldColor(0, 0, 1.0);
	ieee_map.setContainersColor(1.0, 0, 1.0);
	ieee_map.setBotColor(0, 1.0, 0);
	geometry_msgs::Vector3 box_size;
	box_size.x = 4;
	box_size.y = 4;
	box_size.z = 1;
	ieee_map.createBotController(box_size, 6);

	while(ros::ok()){
		ieee_map.publish_field();
		//ieee_map.publish_bot();
		ieee_map.publish_containers();
		ieee_caution.publish_all();
		ros::spinOnce();
		rate.sleep();
	}
}
