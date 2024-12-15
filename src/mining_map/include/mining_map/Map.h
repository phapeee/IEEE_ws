#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <cmath>

#ifndef MAP_H
#define MAP_H

namespace map{
	#define H_PI				(M_PI / 2)
	#define TO_DEGREE			(180.0 / M_PI)
	#define BOT_MAX_LINEAR_VELOCITY 	(3)
	#define BOT_MAX_ANGULAR_VELOCITY 	(0.25)
	#define STOP_RADIUS 			(0.01)
	#define STOP_ANGLE_P 			(0.01)
	#define STOP_ANGLE_N 			(M_PI - 0.01)

	typedef std::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> ProcessFeedback;
	typedef void (*Action)(bool&,bool&);

	struct Segment{
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
		geometry_msgs::Vector3 vector;
		geometry_msgs::Vector3 normal;
		geometry_msgs::Pose normal_vector_pose;
		int normal_direction;

		Segment(geometry_msgs::Point, geometry_msgs::Point, int);
		Segment place(double, double, double, double);
		geometry_msgs::Point getIntersect();
	};

	struct Collision{
		geometry_msgs::Point a;
//		geometry_msgs::Point b;
		Segment* seg1;
		Segment* seg2;
	};

	struct Box{
		Segment* original_walls[4];
		Segment* oriented_walls[4];
		geometry_msgs::Pose pose;

		Box(geometry_msgs::Pose, double);
		void place(geometry_msgs::Pose);
	};

	void default_action(bool& started, bool& isDone){ started = true; isDone = true; } 

	struct CheckPoint {
		geometry_msgs::Pose destination;
		bool action_done = false;
		bool destination_action_done = false;
		bool action_started = false;
		bool destination_action_started = false;
		bool arrived = false;
		Action action;
		Action destination_action;
	};

	class Map{
		protected:
			const static uint8_t LEFT_WALL = 0;
			const static uint8_t TOP_WALL = 1;
			const static uint8_t RIGHT_WALL = 2;
			const static uint8_t BOTTOM_WALL = 3;
			const static uint8_t OUTER_BOTTOM_WALL = 4;
			const static uint8_t MID_BOTTOM_WALL = 5;
			const static uint8_t INNER_BOTTOM__WALL = 6;
			const static uint8_t OUTER_TOP_WALL = 7;
			const static uint8_t MID_TOP_WALL = 8;
			const static uint8_t INNER_TOP_WALL = 9;
			const static uint8_t WALL_COUNT = 14;
			const static uint8_t CONTAINER_COUNT = 2;
			const static uint8_t OBJECTS_COUNT = 4;
			const static uint8_t FIELD = 0;
			const static uint8_t CONTAINER1 = 1;
			const static uint8_t CONTAINER2 = 2;
			const static uint8_t BOT = 3;
			const static uint8_t NORTH_TAG_ID = 1;
			const static uint8_t SOUTH_TAG_ID = 2;
			const static uint8_t EAST_TAG_ID = 7;
		private:
			static uint8_t WEST_TAG_ID;
			Segment* Walls[WALL_COUNT];
			Box* Containers[CONTAINER_COUNT];
			Box* Bot;
			Box* Bot_zone;
			geometry_msgs::Vector3 bot_vel;

			ros::Publisher marker_pub;
	                ros::Publisher markerArray_pub;
	                ros::Publisher bot_vel_pub;
	                ros::Subscriber bot_sub;

			visualization_msgs::Marker field;
			visualization_msgs::Marker actual_field;
                	visualization_msgs::Marker bot_marker;
                	visualization_msgs::Marker bot_zone_marker;
			visualization_msgs::Marker collision_point;
			visualization_msgs::Marker path_arrow;
			visualization_msgs::Marker path_line;
                	visualization_msgs::MarkerArray container_markers;
                	visualization_msgs::MarkerArray normal_vector_markers_list[OBJECTS_COUNT];
			visualization_msgs::MarkerArray collision_points;
			visualization_msgs::MarkerArray path_marker;

			std::vector<Collision> collisions;
			std::vector<CheckPoint> path;
			std::vector<unsigned int> checkpoint_list;

			static boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
			static interactive_markers::MenuHandler menu_handler;
			bool debug_mode;
			unsigned int current_checkpoint = 0;
			bool run_path = false;
		public:
			geometry_msgs::Pose original_bot_pose;
			geometry_msgs::Pose original_container0_pose;
			geometry_msgs::Pose original_container1_pose;

			std::function<void(void)> reset;

			Map(double);
			void init(ros::NodeHandle*, std::string id="0", bool debug=false);
			void loop(void);
			void subscribeBot(ros::NodeHandle*, std::string);
			void publishField();
			void setContainersColor(double, double, double, double a=1.0);
			void setContainersWidth(double);
			void setRunPath(bool);
			void makeBoxControl(std::string con_name, geometry_msgs::Pose, geometry_msgs::Vector3, double, ProcessFeedback);
			void createBotController(geometry_msgs::Vector3, double);
			void createContainersController(geometry_msgs::Vector3, double);
			void tfBroadcastBot(ros::NodeHandle*);
			void updateBot(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
			void updateContainer(uint8_t, geometry_msgs::Pose);
			void updateBotMarker();
			void updateContainerMarkers();
			void updatePathMarker();
			void checkCollision();
			void followPath();
			bool moveBot(geometry_msgs::Pose);
			void moveBotMarker(geometry_msgs::Pose);
			void addCheckPoint(unsigned int, tf::Vector3, double, Action action=default_action, Action dest_action = default_action);
			void removeCheckPoint(unsigned int);
			void static startPath(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&, Map*);
			void pseudoMoveBot();
			void static doNothing(bool&, bool&);
			void Reset(void);
	};

	void cal_intersection(Segment*, Segment*, std::vector<Collision>*);
}

#endif
