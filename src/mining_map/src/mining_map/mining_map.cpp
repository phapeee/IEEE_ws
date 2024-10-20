#include <mining_map/Map.h>

namespace map{

                // point1 and point2 determine end points of the segment.
                // vector points from point1 to point 2.
                // normal_direction = 1 (right hand side of the vector), -1 (left hand side of the the vector)
	Segment::Segment(geometry_msgs::Point point1, geometry_msgs::Point point2, int _normal_direction) : p1(point1), p2(point2), normal_direction(_normal_direction){
                        vector.x = point2.x - point1.x;
                        vector.y = point2.y - point1.y;
                        vector.z = 0;
			float temp;
                        switch (normal_direction){
                                case 1:
                                        normal.x = vector.y;
                                        normal.y = -vector.x;
                                        break;
                                case -1:
                                        normal.x = -vector.y;
                                        normal.y = vector.x;
                                        break;
                                default: break;
                   	}

                       	normal.z = 0;
                       	float norm_mag = sqrt(pow(normal.x, 2.0) + pow(normal.y, 2.0));
                       	normal.x /= norm_mag;
                       	normal.y /= norm_mag;

			float half_normal_angle;
			if (vector.x != 0.0) half_normal_angle = acos(normal.x / norm_mag) / 2.0 * (float) (signbit(normal.y) ? -1.0 : 1.0);
			else half_normal_angle = normal.x > 0 ? 0.0 : M_PI / 2.0;
			normal_vector_pose.position.x = point1.x + vector.x / 2.0;
			normal_vector_pose.position.y = point1.y + vector.y / 2.0;
			normal_vector_pose.orientation.z = sin(half_normal_angle);
			normal_vector_pose.orientation.w = cos(half_normal_angle);
                }

	Segment Segment::place(float x, float y, float cos_theta, float sin_theta){
		geometry_msgs::Point _p1;
		geometry_msgs::Point _p2;
		_p1.x = cos_theta * p1.x - sin_theta * p1.y + x;
		_p1.y = sin_theta * p1.x + cos_theta * p1.y + y;
		_p2.x = cos_theta * p2.x - sin_theta * p2.y + x;
		_p2.y = sin_theta * p2.x + cos_theta * p2.y + y;
		Segment new_seg(_p1, _p2, normal_direction);
		return new_seg;
	}

	Box::Box(float x, float y, float r, float orientation){
		float xs[] = {r, -r, -r, r, r};
                float ys[] = {r, r, -r, -r, r};
		for (int i=0; i < 4; i++){
			geometry_msgs::Point p1;
			geometry_msgs::Point p2;
			p1.x = xs[i];
			p2.x = xs[i+1];
			p1.y = ys[i];
			p2.y = ys[i+1];
			original_walls[i] = new Segment(p1, p2, 1);
			oriented_walls[i] = new Segment(p1, p2, 1);
		}
		pose.position.x = x;
		pose.position.y = y;
		pose.orientation.w = 1;
		place(x, y, orientation);
	}

	void Box::place(float x, float y, float angle){
		float cos_theta = cos(angle);
		float sin_theta = sin(angle);
		for (int i=0; i<4; i++){
			Segment rotated_seg = original_walls[i]->place(x, y, cos_theta, sin_theta);
			oriented_walls[i]->p1 = rotated_seg.p1;
			oriented_walls[i]->p2 = rotated_seg.p2;
			oriented_walls[i]->vector = rotated_seg.vector;
			oriented_walls[i]->normal = rotated_seg.normal;
			oriented_walls[i]->normal_vector_pose = rotated_seg.normal_vector_pose;
		}
		pose.position.x = x;
		pose.position.y = y;
		float half_angle = angle / 2.0;
		pose.orientation.w = cos(half_angle);
		pose.orientation.z = sin(half_angle);
	}

        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> Map::server;
        interactive_markers::MenuHandler Map::menu_handler;

	Map::Map(double safe_radius){
		double s = safe_radius;
		double x[] = {s,      s, 93 - s, 93 - s, s,  55.25 - s, 55.25 - s, 61.75 + s, 61.75 + s, 55.25 - s, 55.25 - s, 61.75 + s, 61.75 + s, 89.5 - s, 89.5 - s,  93 - s, 89.5 - s, 89.5 - s, 93 - s};
		double y[] = {s, 45 - s, 45 - s,      s, s,          s,  14.5 + s,  14.5 + s,         s,    45 - s,  30.5 - s,  30.5 - s,    45 - s,        s,  1.5 + s, 1.5 + s,   45 - s, 43.5 - s, 43.5 - s};
		int n[] = {1, 1, 1, 1, 0, -1, -1, -1, 0, 1, 1, 1, 0, -1, -1, 0, 1, 1};
		int index;
		for (int i=0; i < 18; i++){
			if (i == 4 || i == 8 || i == 12 || i == 15) continue;
			if (i < 4) index = i;
			else if (i < 8) index = i - 1;
			else if (i < 12) index = i - 2;
			else if (i < 15) index = i - 3;
			else index = i - 4;
			geometry_msgs::Point p1;
			geometry_msgs::Point p2;
			p1.x = x[i];
			p1.y = y[i];
			p2.x = x[i+1];
			p2.y = y[i+1];
			p1.z = p2.z = 0;;
			Walls[index] = new Segment(p1, p2, n[i]);
		}

		Bot = new Box(32, 6, 6 + s);
		Containers[0] = new Box(52.25, 3, 3 + s);
		Containers[1] = new Box(27, 42, 3 + s);
	}

	void Map::init_visualization(ros::NodeHandle* nh, std::string id){
		std::string topic_name = "IEEE_map_";
		topic_name += id;
		marker_pub = nh->advertise<visualization_msgs::Marker>(topic_name.c_str(), 10);
		topic_name += "_normal_vectors";
		markerArray_pub = nh->advertise<visualization_msgs::MarkerArray>(topic_name.c_str(), 10);
		Map::server.reset( new interactive_markers::InteractiveMarkerServer("bot_controls","",false) );

		ros::Duration(0.1).sleep();

		field.header.frame_id = "world_map";
	 	field.header.stamp = ros::Time::now();

		std::string ns = "field";
		ns += id;
		field.ns =  ns.c_str();
		field.action = visualization_msgs::Marker::ADD;
		field.type = visualization_msgs::Marker::LINE_LIST;
		field.pose.orientation.w = 1.0;

		visualization_msgs::Marker container_marker;
		visualization_msgs::Marker normal_vector_marker;
		normal_vector_marker = container_marker = bot_marker = field;

		normal_vector_marker.type = visualization_msgs::Marker::ARROW;
		normal_vector_marker.scale.x = 3.0;
		normal_vector_marker.scale.y = 0.2;
		normal_vector_marker.scale.z = 0.2;
		normal_vector_marker.color.r = 1.0;
		normal_vector_marker.color.g = 0.65;
		normal_vector_marker.color.b = 0.0;
		normal_vector_marker.color.a = 1.0;

		setFieldColor(1.0, 1.0, 0);
		setBotColor(1.0, 1.0, 0);

		setFieldWidth(0.5);
		setBotWidth(0.5);

		field.id = 0;
		bot_marker.id = 1;
		container_marker.id = 2;

		for (int i=0; i < WALL_COUNT; i++){
			field.points.push_back(Walls[i]->p1);
			field.points.push_back(Walls[i]->p2);
		}

		for (int i=0; i < 2; i++){
			for (int j=0; j<4; j++){
				container_marker.points.push_back(Containers[i]->oriented_walls[j]->p1);
				container_marker.points.push_back(Containers[i]->oriented_walls[j]->p2);
				if (i == 0){
					bot_marker.points.push_back(Bot->oriented_walls[j]->p1);
					bot_marker.points.push_back(Bot->oriented_walls[j]->p2);
				}
			}
			container_markers.markers.push_back(container_marker);
			container_marker.points.clear();
			container_marker.id++;
		}

		setContainersWidth(0.5);
		setContainersColor(1.0, 1.0, 0);

		int marker_count = 10;
		for (int i=0; i < WALL_COUNT; i++){
			normal_vector_marker.pose = Walls[i]->normal_vector_pose;
			normal_vector_marker.id = marker_count++;
			normal_vector_markers.markers.push_back(normal_vector_marker);
		}
		for (int i=0; i < CONTAINER_COUNT; i++){
			for (int j=0; j<4; j++){
				normal_vector_marker.pose = Containers[i]->oriented_walls[j]->normal_vector_pose;
				normal_vector_marker.id = marker_count++;
				normal_vector_markers.markers.push_back(normal_vector_marker);
				if (i == 0){
					normal_vector_marker.pose = Bot->oriented_walls[j]->normal_vector_pose;
					normal_vector_marker.id = marker_count++;
					normal_vector_markers.markers.push_back(normal_vector_marker);
				}
			}
		}
	}

	void Map::publish_bot(){
		marker_pub.publish(bot_marker);
	}

	void Map::publish_containers(){
		markerArray_pub.publish(container_markers);
	}

	void Map::publish_field(){
		marker_pub.publish(field);
	}

	void Map::publish_debugger(){
		markerArray_pub.publish(normal_vector_markers);
	}

	void Map::publish_all(){
		publish_bot();
		publish_containers();
		publish_field();
		publish_debugger();
	}

	void Map::setFieldColor(float r, float g, float b, float a){
		field.color.r = r;
		field.color.g = g;
		field.color.b = b;
		field.color.a = a;
	}

	void Map::setFieldWidth(float width){
		field.scale.x = width;
	}

	void Map::setContainersWidth(float width){
		for (int i=0; i<CONTAINER_COUNT; i++){
			container_markers.markers[i].scale.x = width;
		}
	}

	void Map::setContainersColor(float r, float g, float b, float a){
		for (int i=0; i<CONTAINER_COUNT; i++){
			container_markers.markers[i].color.r = r;
			container_markers.markers[i].color.g = g;
			container_markers.markers[i].color.b = b;
			container_markers.markers[i].color.a = a;
		}
	}

	void Map::setBotWidth(float width){
		bot_marker.scale.x = width;
	}

	void Map::setBotColor(float r, float g, float b, float a){
		bot_marker.color.r = r;
		bot_marker.color.g = g;
		bot_marker.color.b = b;
		bot_marker.color.a = a;
	}

	void Map::makeBoxControl(visualization_msgs::Marker &marker, geometry_msgs::Pose pose, geometry_msgs::Vector3 box_size, float controller_scale)
	{
		visualization_msgs::InteractiveMarker int_bot;
		int_bot.header.frame_id = "world_map";
		tf::pointTFToMsg(tf::Vector3(pose.position.x, pose.position.y, pose.position.z), int_bot.pose.position);
		int_bot.pose.orientation = pose.orientation;
		int_bot.scale = controller_scale;
		int_bot.name = "Bot";

		visualization_msgs::Marker control_box;
		control_box.type = visualization_msgs::Marker::CUBE;
		control_box.scale = box_size;
		control_box.color.r = 0.5;
		control_box.color.g = 0.5;
		control_box.color.b = 0.5;
		control_box.color.a = 1;

		visualization_msgs::InteractiveMarkerControl control_axis;

		control_axis.orientation.w = 1;
		control_axis.orientation.x = 1;
		control_axis.name = "move_x";
		control_axis.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_bot.controls.push_back( control_axis );

		control_axis.orientation.x = 0;
		control_axis.orientation.z = 1;
		control_axis.name = "move_y";
		int_bot.controls.push_back( control_axis );

		control_axis.orientation.z = 0;
		control_axis.orientation.y = 1;
		control_axis.name = "rotate_z";
		control_axis.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_bot.controls.push_back( control_axis );

		control_axis.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
		control_axis.name = "move_plane";
		int_bot.controls.push_back( control_axis );

		control_axis.markers.push_back( marker );
		control_axis.markers.push_back( control_box );
		control_axis.always_visible = true;
		int_bot.controls.push_back( control_axis );

		Map::server->insert(int_bot);
		Map::server->setCallback(int_bot.name, &processFeedback);
		Map::menu_handler.apply( *Map::server, int_bot.name );
		Map::server->applyChanges();
	}

	void Map::createBotController(geometry_msgs::Vector3 box_size, float controller_scale){
		makeBoxControl(bot_marker, Bot->pose, box_size, controller_scale);
	}

	void Map::createContainersController(geometry_msgs::Vector3 box_size, float controller_scale){
		for (int i=0; i<CONTAINER_COUNT; i++){
			makeBoxControl(container_markers.markers[i], Containers[i]->pose, box_size, controller_scale);
		}
	}

	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  //Map::server->applyChanges();
}
}
