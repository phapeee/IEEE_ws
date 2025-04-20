#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
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
	#define M_2PI				(M_PI * 2)
	#define TO_DEGREE			(180.0 / M_PI)

	#define DEFAULT_BOT_MAX_LINEAR_VELOCITY 	(6)
	#define DEFAULT_BOT_MAX_ANGULAR_VELOCITY 	(0.7)
	#define DEFAULT_STOP_RADIUS 			(0.001)
	#define DEFAULT_STOP_ANGLE 			(0.01)
	#define DEFAULT_LINEAR_K			(3)
	#define DEFAULT_ANGULAR_K			(4)

	#define CAPPED_BOT_MAX_LINEAR_VELOCITY 		(10)
	#define CAPPED_BOT_MAX_ANGULAR_VELOCITY 	(1.5)

	struct checkPointData {
		geometry_msgs::Pose destination_pose;
		geometry_msgs::Pose current_pose;
		double desired_angle;
		uint8_t id;
		bool arrived = false;
	};

	typedef std::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> ProcessFeedback;
	typedef void (*Action)(bool&,bool&,checkPointData&);

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

	struct CheckPoint {
//		geometry_msgs::Pose destination;
		checkPointData cpData;
		bool action_done = false;
		bool destination_action_done = false;
		bool action_started = false;
		bool destination_action_started = false;
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
			double BOT_MAX_LINEAR_VELOCITY = DEFAULT_BOT_MAX_LINEAR_VELOCITY;
            double BOT_MAX_ANGULAR_VELOCITY = DEFAULT_BOT_MAX_ANGULAR_VELOCITY;
            double STOP_RADIUS = DEFAULT_STOP_RADIUS;
            double STOP_ANGLE = DEFAULT_STOP_ANGLE;
            double LINEAR_K = DEFAULT_LINEAR_K;
            double ANGULAR_K = DEFAULT_ANGULAR_K;

			bool autoSpeedControl = false;

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

			static std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
			static interactive_markers::MenuHandler menu_handler;
			bool debug_mode;
			unsigned int current_checkpoint = 0;
		public:
			bool run_path = false;
			geometry_msgs::Pose original_bot_pose;
			geometry_msgs::Pose original_container0_pose;
			geometry_msgs::Pose original_container1_pose;

			std::function<void(void)> reset;

			Map(double);
			void init(ros::NodeHandle*, std::string id="0", bool debug=false);
			void subscribeBot(ros::NodeHandle*, std::string);
			void publishField();
			void setContainersColor(double, double, double, double a=1.0);
			void setContainersWidth(double);
			void setRunPath(bool);
			void makeBoxControl(std::string con_name, geometry_msgs::Pose, geometry_msgs::Vector3, double, ProcessFeedback);
			void createContainersController(geometry_msgs::Vector3, double);
			void tfBroadcastBot(ros::NodeHandle*);
			void updateBot(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
			void updateContainer(uint8_t, geometry_msgs::Pose);
			void updateBotMarker();
			void updateContainerMarkers();
			void updatePathMarker();
			void followPath();
			bool moveBot(geometry_msgs::Pose);
			void moveBotMarker(geometry_msgs::Pose);
			void addCheckPoint(unsigned int, tf::Vector3, double, Action, Action);
			void removeCheckPoint(unsigned int);
			void static startPath(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&, Map*);
			void pseudoMoveBot();
			void static doNothing(bool&, bool&, checkPointData&);
			void Reset(void);
			void setLinK(double);
			void setAngK(double);
			void setStopDist(double);
			void setMaxLinVel(double);
			void setMaxAngVel(double);
			void resetLinK(void);
			void resetAngK(void);
			void resetStopDist(void);
			void resetMaxLinVel(void);
			void resetMaxAngVel(void);
			void enableAutoSpdCtrl(void);
			void disableAutoSpdCtrl(void);
			void manualSpeedControl(geometry_msgs::Vector3);
			geometry_msgs::Pose getBotPose();
	};

	void cal_intersection(Segment*, Segment*, std::vector<Collision>*);
}

#endif
