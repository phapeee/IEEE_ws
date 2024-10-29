#include <mining_map/Map.h>

namespace map{

                // point1 and point2 determine end points of the segment.
                // vector points from point1 to point 2.
                // normal_direction = 1 (right hand side of the vector), -1 (left hand side of the the vector)
	Segment::Segment(geometry_msgs::Point point1, geometry_msgs::Point point2, int _normal_direction) : p1(point1), p2(point2), normal_direction(_normal_direction){
                        vector.x = point2.x - point1.x;
                        vector.y = point2.y - point1.y;
                        vector.z = 0;
			double temp;
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
                       	double norm_mag = sqrt(pow(normal.x, 2.0) + pow(normal.y, 2.0));
                       	normal.x /= norm_mag;
                       	normal.y /= norm_mag;

			double half_normal_angle;
			if (vector.x != 0.0) half_normal_angle =  acos(normal.x) / 2.0 * (signbit(normal.y) ? -1.0 : 1.0);
			else half_normal_angle = normal.x > 0 ? 0.0 : M_PI / 2.0;
			normal_vector_pose.position.x = point1.x + vector.x / 2.0;
			normal_vector_pose.position.y = point1.y + vector.y / 2.0;
			normal_vector_pose.orientation.z = sin(half_normal_angle);
			normal_vector_pose.orientation.w = cos(half_normal_angle);
                }

	Segment Segment::place(double x, double y, double cos_theta, double sin_theta){
		geometry_msgs::Point _p1;
		geometry_msgs::Point _p2;
		_p1.x = cos_theta * p1.x - sin_theta * p1.y + x;
		_p1.y = sin_theta * p1.x + cos_theta * p1.y + y;
		_p2.x = cos_theta * p2.x - sin_theta * p2.y + x;
		_p2.y = sin_theta * p2.x + cos_theta * p2.y + y;
		Segment new_seg(_p1, _p2, normal_direction);
		return new_seg;
	}

	Box::Box(geometry_msgs::Pose _pose, double r){
		double xs[] = {r, -r, -r, r, r};
                double ys[] = {r, r, -r, -r, r};
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
		place(_pose);
	}

	void Box::place(geometry_msgs::Pose _pose){
		double angle = asin(_pose.orientation.z) * 2.0;
		double cos_theta = cos(angle);
		double sin_theta = sin(angle);
		for (int i=0; i<4; i++){
			Segment rotated_seg = original_walls[i]->place(_pose.position.x, _pose.position.y, cos_theta, sin_theta);
			oriented_walls[i]->p1 = rotated_seg.p1;
			oriented_walls[i]->p2 = rotated_seg.p2;
			oriented_walls[i]->vector = rotated_seg.vector;
			oriented_walls[i]->normal = rotated_seg.normal;
			oriented_walls[i]->normal_vector_pose = rotated_seg.normal_vector_pose;
		}
		pose = _pose;
	}

        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> Map::server;
        interactive_markers::MenuHandler Map::menu_handler;

	Map::Map(double safe_radius){
		double s = safe_radius;
		double x[] = {s,      s, 93 - s, 93 - s, s,  55.25 - s, 55.25 - s, 61.75 + s, 61.75 + s, 55.25 - s, 55.25 - s, 61.75 + s, 61.75 + s, 89.5 - s, 89.5 - s,  93 - s, 89.5 - s, 89.5 - s, 93 - s};
		double y[] = {s, 45 - s, 45 - s,      s, s,          s,  14.5 + s,  14.5 + s,         s,    45 - s,  30.5 - s,  30.5 - s,    45 - s,        s,  1.5 + s, 1.5 + s,   45 - s, 43.5 - s, 43.5 - s};
		int n[] = {1, 1, 1, 1, 0, -1, -1, -1, 0, 1, 1, 1, 0, -1, -1, 0, 1, 1};
		int index;
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
		for (int i=0; i < 18; i++){
			if (i == 4 || i == 8 || i == 12 || i == 15) continue;
			if (i < 4) index = i;
			else if (i < 8) index = i - 1;
			else if (i < 12) index = i - 2;
			else if (i < 15) index = i - 3;
			else index = i - 4;
			p1.x = x[i];
			p1.y = y[i];
			p2.x = x[i+1];
			p2.y = y[i+1];
			p1.z = p2.z = 0;;
			Walls[index] = new Segment(p1, p2, n[i]);
		}

		geometry_msgs::Pose _pose;
		_pose.position.x = 32;
		_pose.position.y = 6;
		_pose.orientation.w = 1.0;
		original_bot_pose = _pose;
		Bot = new Box(_pose, 6);
		Bot_zone = new Box(_pose, 6 + s);
		_pose.position.x = 52.25;
		_pose.position.y = 3;
		original_container0_pose = _pose;
		Containers[0] = new Box(_pose, 3 + s);
		_pose.position.x = 27;
		_pose.position.y = 42;
		original_container1_pose = _pose;
		Containers[1] = new Box(_pose, 3 + s);

		debug_mode = false;
	}

	void Map::init(ros::NodeHandle* nh, std::string id, bool debug){

		debug_mode = debug;
		bot_pub = nh->advertise<geometry_msgs::Pose>("IMU_sensor", 10);
		bot_vel_pub = nh->advertise<geometry_msgs::Vector3>("Bot_Velocities", 10);

		if (!debug) return;

		std::string topic_name = "IEEE_map_";
		topic_name += id;
		marker_pub = nh->advertise<visualization_msgs::Marker>(topic_name.c_str(), 1);

		topic_name += "_normal_vectors";
		markerArray_pub = nh->advertise<visualization_msgs::MarkerArray>(topic_name.c_str(), 1);

		Map::server.reset( new interactive_markers::InteractiveMarkerServer("bot_controls","",false) );

		ros::Duration(0.1).sleep();

		field.header.frame_id = "world_map";
	 	//field.header.stamp = ros::Time::now();

		std::string ns = "field";
		ns += id;
		field.ns =  ns.c_str();
		field.action = visualization_msgs::Marker::ADD;
		field.type = visualization_msgs::Marker::LINE_LIST;
		field.pose.orientation.w = 1.0;
		field.lifetime = ros::Duration();

		visualization_msgs::Marker container_marker;
		visualization_msgs::Marker normal_vector_marker;
		visualization_msgs::MarkerArray normal_vector_markers;
		normal_vector_marker = container_marker = bot_marker = collision_point = path_line = actual_field = bot_zone_marker = field;

		path_line.type = visualization_msgs::Marker::LINE_STRIP;
		path_line.scale.x = 0.2;
		path_line.color.r = 0.5;
		path_line.color.b = 0.5;
		path_line.color.a = 1.0;

		collision_point.type = visualization_msgs::Marker::SPHERE;
		collision_point.color.r = 1.0;
		collision_point.color.a = 1.0;
		collision_point.scale.x = 0.5;
		collision_point.scale.y = 0.5;
		collision_point.scale.z = 0.5;
		collision_point.lifetime = ros::Duration(0.2);

		normal_vector_marker.type = visualization_msgs::Marker::ARROW;
		normal_vector_marker.scale.x = 3.0;
		normal_vector_marker.scale.y = 0.2;
		normal_vector_marker.scale.z = 0.2;
		normal_vector_marker.color.r = 1.0;
		normal_vector_marker.color.g = 0.65;
		normal_vector_marker.color.b = 0.0;
		normal_vector_marker.color.a = 1.0;

		path_arrow = normal_vector_marker;
		path_arrow.color.r = 0.0;
		path_arrow.color.g = 1.0;
		path_arrow.color.b = 1.0;
		path_arrow.color.a = 1.0;

		bot_marker.scale.x = 0.5;
		bot_marker.color.r = 0.0;
		bot_marker.color.g = 1.0;
		bot_marker.color.b = 0.0;
		bot_marker.color.a = 1.0;

		bot_zone_marker.scale.x = 0.25;
		bot_zone_marker.color.r = 1.0;
		bot_zone_marker.color.g = 1.0;
		bot_zone_marker.color.b = 0.0;
		bot_zone_marker.color.a = 1.0;

		field.scale.x = 0.25;
		field.color.r = 1.0;
		field.color.g = 1.0;
		field.color.b = 0.0;
		field.color.a = 1.0;

		container_marker.scale.x = 0.25;
		container_marker.color.r = 1.0;
		container_marker.color.g = 1.0;
		container_marker.color.b = 0.0;
		container_marker.color.a = 1.0;

		double x[] = {0,  0, 93, 93, 0,  55.25, 55.25, 61.75, 61.75, 55.25, 55.25, 61.75, 61.75, 89.5, 89.5,  93, 89.5, 89.5, 93};
		double y[] = {0, 45, 45,  0, 0,      0,  14.5,  14.5,     0,    45,  30.5,  30.5,    45,    0,  1.5, 1.5,   45, 43.5, 43.5};
		int n[] = {1, 1, 1, 1, 0, -1, -1, -1, 0, 1, 1, 1, 0, -1, -1, 0, 1, 1};
		int index;
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
		for (int i=0; i < 18; i++){
			if (i == 4 || i == 8 || i == 12 || i == 15) continue;
			if (i < 4) index = i;
			else if (i < 8) index = i - 1;
			else if (i < 12) index = i - 2;
			else if (i < 15) index = i - 3;
			else index = i - 4;
			p1.x = x[i];
			p1.y = y[i];
			p2.x = x[i+1];
			p2.y = y[i+1];
			p1.z = p2.z = 0;
			actual_field.points.push_back(p1);
			actual_field.points.push_back(p2);
		}

		actual_field.scale.x = 0.5;
		actual_field.color.r = 0.0;
		actual_field.color.g = 0.0;
		actual_field.color.b = 1.0;
		actual_field.color.a = 1.0;

		actual_field.id = 0;
		field.id = 1;
		bot_marker.id = 2;
		bot_zone_marker.id = 3;
		path_line.id = 4;
		container_marker.id = 5;

		for (int i=0; i < WALL_COUNT; i++){
			field.points.push_back(Walls[i]->p1);
			field.points.push_back(Walls[i]->p2);
		}

		for (int i=0; i < CONTAINER_COUNT; i++){
			container_marker.id += i;
			container_markers.markers.push_back(container_marker);
		}

		int marker_count = 10;
		for (int i=0; i < WALL_COUNT; i++){
			normal_vector_marker.pose = Walls[i]->normal_vector_pose;
			normal_vector_marker.id = marker_count++;
			normal_vector_markers.markers.push_back(normal_vector_marker);
		}
		normal_vector_markers_list[FIELD] = normal_vector_markers;
		normal_vector_markers.markers.clear();
		for (int i=0; i < CONTAINER_COUNT; i++){
			for (int j=0; j<4; j++){
				normal_vector_marker.id = marker_count++;
				normal_vector_markers.markers.push_back(normal_vector_marker);
			}
			normal_vector_markers_list[CONTAINER1+i] = normal_vector_markers;
			normal_vector_markers.markers.clear();
		}
		for (int i=0; i<4; i++){
			normal_vector_marker.id = marker_count++;
			normal_vector_markers.markers.push_back(normal_vector_marker);
		}
		normal_vector_marker.scale.x = 5.0;
		normal_vector_marker.scale.y = 0.5;
		normal_vector_marker.scale.z = 0.5;
		normal_vector_marker.color.r = 0.0;
		normal_vector_marker.color.b = 0.0;
		normal_vector_marker.color.g = 1.0;
		normal_vector_marker.id = marker_count++;
		normal_vector_markers.markers.push_back(normal_vector_marker);
		normal_vector_markers_list[BOT] = normal_vector_markers;

		updateContainerMarkers();
		updateBotMarker();

		auto startCallback = [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb){
			this->setRunPath(true);
		};
		auto stopCallback = [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb){
			this->setRunPath(false);
		};
		auto resetCallback = [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb){
			visualization_msgs::InteractiveMarker int_marker;
			moveBotMarker(original_bot_pose);
			if (Map::server->get("container_0", int_marker)){
				int_marker.pose = original_container0_pose;
				Map::server->erase("container_0");
				Map::server->insert(int_marker);
				updateContainer(0, original_container0_pose);
			}
			if (Map::server->get("container_1", int_marker)){
				int_marker.pose = original_container1_pose;
				Map::server->erase("container_1");
				Map::server->insert(int_marker);
				updateContainer(1, original_container1_pose);
			}
			Map::server->applyChanges();

			current_checkpoint = 0;
			for (auto &p : path){
				p.action_done = false;
				p.destination_action_done = false;
				p.action_started = false;
				p.destination_action_started = false;
				p.arrived = false;
			}
			bot_vel.x = 0.0;
			bot_vel.y = 0.0;
			bot_vel.z = 0.0;
			setRunPath(false);
			publishField();
			markerArray_pub.publish(path_marker);
		};

		tf::Vector3 pos(27, -5, 0);

		visualization_msgs::InteractiveMarker int_marker;
		int_marker.header.frame_id = "world_map";
		int_marker.name = "start_button";
		int_marker.description = "Start";
		int_marker.scale = 5;
		tf::pointTFToMsg(pos, int_marker.pose.position);

		visualization_msgs::Marker button;
		button.type = visualization_msgs::Marker::SPHERE;
		button.scale.x = 5;
		button.scale.y = 5;
		button.scale.z = 5;
		button.color.g = 1.0;
		button.color.a = 1.0;
		button.id = 7;

		visualization_msgs::InteractiveMarkerControl control;
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
		control.name = "button_control";
		control.always_visible = true;
		control.markers.push_back(button);
		int_marker.controls.push_back(control);

		Map::server->insert(int_marker);
		Map::server->setCallback(int_marker.name, startCallback);

		pos.setX(37);
		tf::pointTFToMsg(pos, int_marker.pose.position);
		int_marker.name = "stop_button";
		int_marker.description = "Stop";
		int_marker.controls[0].markers[0].id++;
		int_marker.controls[0].markers[0].color.g = 0.0;
		int_marker.controls[0].markers[0].color.r = 1.0;

		Map::server->insert(int_marker);
		Map::server->setCallback(int_marker.name, stopCallback);

		pos.setX(47);
		tf::pointTFToMsg(pos, int_marker.pose.position);
		int_marker.name = "reset_button";
		int_marker.description = "Reset";
		int_marker.controls[0].markers[0].id++;
		int_marker.controls[0].markers[0].color.b = 0.5;
		int_marker.controls[0].markers[0].color.g = 0.5;
		int_marker.controls[0].markers[0].color.r = 0.5;

		Map::server->insert(int_marker);
		Map::server->setCallback(int_marker.name, resetCallback);
	}

	void Map::updateBot(geometry_msgs::Pose _pose){
		Bot->place(_pose);
		Bot_zone->place(_pose);
		if (debug_mode) updateBotMarker();
	}

	void Map::updateContainer(uint8_t num, geometry_msgs::Pose _pose){
		if (num != 0 && num != 1) return;
		Containers[num]->place(_pose);
		if (debug_mode) updateContainerMarkers();
	}

	void Map::updateBotMarker(){
		bot_marker.points.clear();
		bot_zone_marker.points.clear();
		for (int i=0; i<4; i++){
			bot_marker.points.push_back(Bot->oriented_walls[i]->p1);
			bot_marker.points.push_back(Bot->oriented_walls[i]->p2);

			bot_zone_marker.points.push_back(Bot_zone->oriented_walls[i]->p1);
			bot_zone_marker.points.push_back(Bot_zone->oriented_walls[i]->p2);

			normal_vector_markers_list[BOT].markers[i].pose = Bot_zone->oriented_walls[i]->normal_vector_pose;
		}
		normal_vector_markers_list[BOT].markers[4].pose = Bot->pose;
		normal_vector_markers_list[BOT].markers[4].pose.position.z = 1.0;

		marker_pub.publish(bot_marker);
		marker_pub.publish(bot_zone_marker);
		markerArray_pub.publish(normal_vector_markers_list[BOT]);
	}

	void Map::updateContainerMarkers(){
		for (int i=0; i < CONTAINER_COUNT; i++){
			container_markers.markers[i].points.clear();
                        for (int j=0; j<4; j++){
				container_markers.markers[i].points.push_back(Containers[i]->oriented_walls[j]->p1);
				container_markers.markers[i].points.push_back(Containers[i]->oriented_walls[j]->p2);
				normal_vector_markers_list[CONTAINER1+i].markers[j].pose = Containers[i]->oriented_walls[j]->normal_vector_pose;
                        }
                }
		markerArray_pub.publish(container_markers);
		markerArray_pub.publish(normal_vector_markers_list[CONTAINER1]);
		markerArray_pub.publish(normal_vector_markers_list[CONTAINER2]);
	}

	void Map::updatePathMarker(){
		path_marker.markers.clear();
		path_line.points.clear();
		path_arrow.id = 100;
		for (int i=0; i < path.size(); i++){
			geometry_msgs::Point p;
			p.x = path[i].destination.position.x;
			p.y = path[i].destination.position.y;
			p.z = path[i].destination.position.z;
			path_line.points.push_back(p);

			path_arrow.id++;
			path_arrow.pose = path[i].destination;
			path_marker.markers.push_back(path_arrow);
		}
		path_marker.markers.push_back(path_line);
		markerArray_pub.publish(path_marker);
	}

	void Map::publishField(){
		if (!debug_mode) return;
		marker_pub.publish(field);
		marker_pub.publish(actual_field);
		markerArray_pub.publish(normal_vector_markers_list[FIELD]);
	}

	void Map::makeBoxControl(std::string con_name, geometry_msgs::Pose pose, geometry_msgs::Vector3 box_size, double controller_scale, ProcessFeedback processFeedback)
	{
		visualization_msgs::InteractiveMarker int_bot;
		int_bot.header.frame_id = "world_map";
		tf::pointTFToMsg(tf::Vector3(pose.position.x, pose.position.y, pose.position.z), int_bot.pose.position);
		int_bot.pose.orientation = pose.orientation;
		int_bot.scale = controller_scale;
		int_bot.name = con_name;
		int_bot.description = con_name;

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

		control_axis.name = "move_plane";
		control_axis.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
		int_bot.controls.push_back( control_axis );

		control_axis.name = "marker";
		control_axis.markers.push_back( control_box );
		control_axis.always_visible = true;
		int_bot.controls.push_back( control_axis );

		Map::server->insert(int_bot);
		Map::server->setCallback(int_bot.name, processFeedback);
		Map::menu_handler.apply( *Map::server, int_bot.name );
		Map::server->applyChanges();
	}

	void Map::createBotController(geometry_msgs::Vector3 box_size, double controller_scale){
		auto controllerCallback = [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb){
			this->bot_pub.publish(fb->pose);
		};
		makeBoxControl("Bot", Bot->pose, box_size, controller_scale, controllerCallback);\
	}

	void Map::createContainersController(geometry_msgs::Vector3 box_size, double controller_scale){
		auto controllerCallback0 = [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb){
			this->updateContainer(0, fb->pose);
		};
		auto controllerCallback1 = [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb){
			this->updateContainer(1, fb->pose);
		};

		makeBoxControl("container_0", Containers[0]->pose, box_size, controller_scale, controllerCallback0);
		makeBoxControl("container_1", Containers[1]->pose, box_size, controller_scale, controllerCallback1);
	}

	void Map::checkCollision(){
		collisions.clear();
		for (int i=0; i<4; i++){
			for (int j=0; j<WALL_COUNT; j++){
				cal_intersection(Bot_zone->oriented_walls[i], Walls[j], &collisions);
			}
			for (int j=0; j<CONTAINER_COUNT; j++){
				for (int k=0; k<4; k++){
					cal_intersection(Bot_zone->oriented_walls[i], Containers[j]->oriented_walls[k], &collisions);
				}
			}
		}
		if (debug_mode){
			collision_points.markers.clear();
			collision_point.id = 50;
			for (const auto& c : collisions){
				collision_point.pose.position.x = c.a.x;
				collision_point.pose.position.y = c.a.y;
				collision_point.pose.position.z = c.a.z;
				collision_point.id++;
				collision_points.markers.push_back(collision_point);
			}
			markerArray_pub.publish(collision_points);
		}
	}

	void Map::followPath(){
		if (!run_path || path.size() == 0 || current_checkpoint >= path.size()) {
			setRunPath(false);
			return;
		}

		if (!path[current_checkpoint].arrived) path[current_checkpoint].arrived = moveBot(path[current_checkpoint].destination);
		else path[current_checkpoint].destination_action(path[current_checkpoint].destination_action_started, path[current_checkpoint].destination_action_done);

		path[current_checkpoint].action(path[current_checkpoint].action_started, path[current_checkpoint].action_done);
		//ROS_INFO("cp: %d, arrived: %d, action: %d, dest action: %d", (int) current_checkpoint, path[current_checkpoint].arrived, path[current_checkpoint].action_done, path[current_checkpoint].destination_action_done);
		if (path[current_checkpoint].action_done && path[current_checkpoint].destination_action_done) {
			current_checkpoint++;
//			ROS_INFO("DONE!");
		}
	}

	bool Map::moveBot(geometry_msgs::Pose dest){
		geometry_msgs::Vector3 velocity;
		velocity.x = 0.0;
		velocity.y = 0.0;
		velocity.z = 0.0;

		tf::Vector3 dist(dest.position.x - Bot->pose.position.x, dest.position.y - Bot->pose.position.y, 0);
		double angle_dif =  asin(dest.orientation.z) - asin(Bot->pose.orientation.z);
		if (dist.length2() > STOP_RADIUS) {
			dist.normalize();
			dist *= BOT_MAX_LINEAR_VELOCITY;
			velocity.x = dist.x();
			velocity.y = dist.y();
			velocity.z = dist.z();
		}
		if (abs(angle_dif) > STOP_ANGLE_P && abs(angle_dif) < STOP_ANGLE_N) velocity.z = BOT_MAX_ANGULAR_VELOCITY * (signbit(angle_dif) ? (angle_dif < -H_PI ? 1.0 : -1.0) : (angle_dif > H_PI ? -1.0 : 1.0));
		bot_vel_pub.publish(velocity);
		bot_vel = velocity;

		if (velocity.x == 0.0 && velocity.y == 0.0 && velocity.z == 0.0) return true;
		return false;
	}

	void Map::moveBotMarker(geometry_msgs::Pose pose){
		visualization_msgs::InteractiveMarker int_marker;
		if (Map::server->get("Bot", int_marker)){
			int_marker.pose = pose;
			Map::server->erase("Bot");
			Map::server->insert(int_marker);
			Map::server->applyChanges();
			updateBot(pose);
		}
	}

	void Map::pseudoMoveBot(){
		if (!run_path) return;
		geometry_msgs::Pose new_pose;
		new_pose.position.x = (double) Bot->pose.position.x + bot_vel.x / 30.0;
		new_pose.position.y = (double) Bot->pose.position.y + bot_vel.y / 30.0;

		double new_half_angle = (double) asin(Bot->pose.orientation.z) + bot_vel.z / 60.0;
		if (new_half_angle > H_PI) new_half_angle -= M_PI;
		else if (new_half_angle < -H_PI) new_half_angle += M_PI;
		new_pose.orientation.w = (double) cos(new_half_angle);
		new_pose.orientation.z = (double) sin(new_half_angle);
		moveBotMarker(new_pose);
	}

	void Map::addCheckPoint(unsigned int index, tf::Vector3 position, double orientation, Action action, Action dest_action){
		CheckPoint cp;

		cp.destination.position.x = position.x();
		cp.destination.position.y = position.y();
		cp.destination.position.z = 0;

		tfScalar angle = std::min(M_PI, std::max(-M_PI, orientation));
		tf::Vector3 axis(0.0, 0.0, 1.0);
		tf::Quaternion orien(axis, angle);

		cp.destination.orientation.x = orien.x();
		cp.destination.orientation.y = orien.y();
		cp.destination.orientation.z = orien.z();
		cp.destination.orientation.w = orien.w();

		cp.action = action;
		cp.destination_action = dest_action;

		path.insert(path.begin() + index, cp);

		updatePathMarker();

		visualization_msgs::InteractiveMarker int_marker;
		int_marker.header.frame_id = "world_map";
		tf::pointTFToMsg(position, int_marker.pose.position);
		int_marker.scale = 2;
		int_marker.pose.orientation = cp.destination.orientation;
		int_marker.name = "cp_";
		int_marker.name += std::to_string(index);
		int_marker.description = "check point ";
		int_marker.description += std::to_string(index);

		visualization_msgs::Marker control_box;
		control_box.type = visualization_msgs::Marker::SPHERE;
		control_box.scale.x = 2;
		control_box.scale.y = 2;
		control_box.scale.z = 2;
		control_box.color.r = 0.5;
		control_box.color.g = 0.5;
		control_box.color.b = 0.5;
		control_box.color.a = 1;

		visualization_msgs::InteractiveMarkerControl control_axis;

		control_axis.orientation.w = 1;
		control_axis.orientation.x = 1;
		control_axis.name = "move_x";
		control_axis.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back( control_axis );

		control_axis.orientation.x = 0;
		control_axis.orientation.z = 1;
		control_axis.name = "move_y";
		int_marker.controls.push_back( control_axis );

		control_axis.orientation.z = 0;
		control_axis.orientation.y = 1;
		control_axis.name = "rotate_z";
		control_axis.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back( control_axis );

		control_axis.name = "move_plane";
		control_axis.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
		int_marker.controls.push_back( control_axis );

		control_axis.name = "marker";
		control_axis.markers.push_back( control_box );
		control_axis.always_visible = true;
		int_marker.controls.push_back( control_axis );

		auto controllerCallback = [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb){
			auto pos = fb->marker_name.find('_');
			unsigned int index = std::stoi(fb->marker_name.substr(pos + 1, fb->marker_name.length() - pos));
			path[index].destination = fb->pose;
			updatePathMarker();
		};

		Map::server->insert(int_marker);
		Map::server->setCallback(int_marker.name, controllerCallback);
		Map::menu_handler.apply( *Map::server, int_marker.name );
		Map::server->applyChanges();
	}

	void Map::removeCheckPoint(unsigned int index){
		path.erase(path.begin() + index);
	}

	void Map::setRunPath(bool run){
		if (run_path == run) return;
		run_path = run;
		visualization_msgs::InteractiveMarker int_marker;
		if (Map::server->get("Bot", int_marker)){
			if (run_path) {
				int_marker.controls[4].markers[0].color.r = 0;
				int_marker.controls[4].markers[0].color.b = 0;
				int_marker.controls[4].markers[0].color.g = 1;
			}
			else {
				int_marker.controls[4].markers[0].color.r = 1;
				int_marker.controls[4].markers[0].color.g = 0;
			}
			Map::server->erase("Bot");
			Map::server->insert(int_marker);
			Map::server->applyChanges();
		}
	}
	void Map::doNothing(bool& started, bool& done) {
        	started = true;
        	done = true;
	}


	void cal_intersection(Segment* seg1, Segment* seg2, std::vector<Collision>* collisions){
		if (std::min(seg1->p1.x, seg1->p2.x) > std::max(seg2->p1.x, seg2->p2.x) ||
			std::max(seg1->p1.x, seg1->p2.x) < std::min(seg2->p1.x, seg2->p2.x) ||
			std::min(seg1->p1.y, seg1->p2.y) > std::max(seg2->p1.y, seg2->p2.y) ||
                        std::max(seg1->p1.y, seg1->p2.y) < std::min(seg2->p1.y, seg2->p2.y)) return;

		Collision collision;
		collision.seg1 = seg1;
		collision.seg2 = seg2;
		geometry_msgs::Point p;
		p.z = 0;

		double denom = seg2->vector.x * seg1->vector.y - seg1->vector.x * seg2->vector.y;
		if (denom == 0.0) return;
		double y3y1 = seg2->p1.y - seg1->p1.y;
		double x3x1 = seg2->p1.x - seg1->p1.x;
		double num_a = seg2->vector.x * y3y1 - seg2->vector.y * x3x1;
		double num_b = seg1->vector.x * y3y1 - seg1->vector.y * x3x1;
		double alpha = num_a / denom;
		double beta = num_b / denom;
		if (alpha < 0 || alpha > 1 || beta < 0 || beta > 1) return;
		if (alpha < 0.05) collision.a = seg1->p1;
		else if (alpha > 0.95) collision.a = seg1->p2;
		else {
			p.x = seg1->p1.x + seg1->vector.x * alpha;
			p.y = seg1->p1.y + seg1->vector.y * alpha;
			collision.a = p;
		}

		collisions->push_back(collision);
	}
}
