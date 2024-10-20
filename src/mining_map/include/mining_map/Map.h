#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <cmath>

#ifndef MAP_H
#define MAP_H

namespace map{

	struct Segment{
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
		geometry_msgs::Vector3 vector;
		geometry_msgs::Vector3 normal;
		geometry_msgs::Pose normal_vector_pose;
		int normal_direction;

		Segment(geometry_msgs::Point, geometry_msgs::Point, int);
		Segment place(float, float, float, float);
		geometry_msgs::Point getIntersect();
	};

	struct Intersection{
		float a;
		float b;
		Segment* seg1;
		Segment* seg2;
	};

	struct Box{
		Segment* original_walls[4];
		Segment* oriented_walls[4];
		geometry_msgs::Pose pose;

		Box(float, float, float, float orientation=0);
		void place(float, float, float);
	};

	class Map{
		protected:
			const uint8_t LEFT_WALL = 0;
			const uint8_t TOP_WALL = 1;
			const uint8_t RIGHT_WALL = 2;
			const uint8_t BOTTOM_WALL = 3;
			const uint8_t OUTER_BOTTOM_WALL = 4;
			const uint8_t MID_BOTTOM_WALL = 5;
			const uint8_t INNER_BOTTOM__WALL = 6;
			const uint8_t OUTER_TOP_WALL = 7;
			const uint8_t MID_TOP_WALL = 8;
			const uint8_t INNER_TOP_WALL = 9;
			const static uint8_t WALL_COUNT = 14;
			const static uint8_t CONTAINER_COUNT = 2;
		private:
			Segment* Walls[WALL_COUNT];
			Box* Containers[CONTAINER_COUNT];
			Box* Bot;

	                ros::Publisher marker_pub;
	                ros::Publisher markerArray_pub;

			visualization_msgs::Marker field;
                	visualization_msgs::MarkerArray container_markers;
                	visualization_msgs::Marker bot_marker;
                	visualization_msgs::MarkerArray normal_vector_markers;
		public:
			static boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
			static interactive_markers::MenuHandler menu_handler;

			Map(double);
			static Intersection calculate_intersection(Segment*, Segment*);
			void init_visualization(ros::NodeHandle*, std::string id="0");
			void publish_bot();
			void publish_containers();
			void publish_field();
			void publish_debugger();
			void publish_all();
			void setFieldColor(float, float, float, float a=1.0);
			void setFieldWidth(float);
			void setContainersColor(float, float, float, float a=1.0);
			void setContainersWidth(float);
			void setBotColor(float, float, float, float a=1.0);
			void setBotWidth(float);
			//void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
			void makeBoxControl(visualization_msgs::Marker &marker, geometry_msgs::Pose, geometry_msgs::Vector3, float);
			void createBotController(geometry_msgs::Vector3, float);
			void createContainersController(geometry_msgs::Vector3, float);
	};
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
}

#endif
