#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <mining_map/Map.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt16.h>
#include <cmath>
#include <wiringPi.h>
#include <serial/serial.h>
#include <sstream>
#include <string>
#include <std_srvs/Empty.h>
#include "april_tag_detection/GetAprilTag.h"
#include <algorithm>
#include <tof_sensor/Tof.h>
#include <std_msgs/Float64MultiArray.h>
#include <mecanum_ik/adjustPositionSrv.h>

#define CMD_GATHER_IN (0)
#define CMD_GATHER_OUT (10)
#define CMD_GATHER_STOP (20)

#define CMD_LEFT_BELT_IN (1)
#define CMD_LEFT_BELT_OUT (11)
#define CMD_LEFT_BELT_STOP (21)

#define CMD_RIGHT_BELT_IN (2)
#define CMD_RIGHT_BELT_OUT (12)
#define CMD_RIGHT_BELT_STOP (22)

#define CMD_MID_BELT_IN (3)
#define CMD_MID_BELT_OUT (13)
#define CMD_MID_BELT_STOP (23)

#define CMD_RUN_SORTER (4)
#define CMD_STOP_SORTER (24)

#define CMD_GEO_GRAB (5)
#define CMD_GEO_UNGRAB (15)

#define CMD_NEB_GRAB (6)
#define CMD_NEB_UNGRAB (16)

#define CMD_GEO_DUMP (7)
#define CMD_GEO_PUSH (17)

#define CMD_NEB_DUMP (8)
#define CMD_NEB_PUSH (18)

#define CMD_PLACE_BEACON (9)
#define CMD_RESET_BEACON (19)

#define CMD_FLOOD_LIGHT_ON (30)
#define CMD_FLOOD_LIGHT_OFF (40)

#define CMD_DEPLOY_GATHERING (31)

#define M_2PI (M_PI * 2)
#define NO_MOVE (-99)

#define ZIG_ZAC_WIDTH (0.5)
#define ZIG_ZAC_HEIGHT (0.25)
#define ZIG_ZAC_LOOP (2)

#define TARGET_BEACON_X (0)
#define TARGET_BEACON_Z (6.5)
#define TRUE_TARGET_BEACON_Y (3.2)
#define TARGET_BEACON_THRESHOLD (0.02)
#define MAX_VEL (10.0)

#define TOF_RIGHT (0)
#define TOF_BACK (1)
#define TOF_LEFT (2)
#define MM_TO_INCH (0.0393700787)
#define TOF_BEACON_DIST (94*MM_TO_INCH)
#define TOF_ERROR_THRESHOLD (0.1)

#define LIGHT_DETECTOR_PIN (24)

#define SOUTH_TAG_POS_X (43.3)
#define SOUTH_TAG_POS_Y (0)

#define GEO_CSC_SENSOR_PIN (22)
#define NEB_CSC_SENSOR_PIN (27)

serial::Serial serial_port;
ros::ServiceClient mecanum_ik_client;
ros::ServiceClient april_tag_client;
ros::ServiceClient tof_client;
ros::ServiceClient mecanum_ik_adjust_pos_client;
ros::Publisher bot_vel_sub;
bool arduino_active;
bool tof_active;
bool tof_new = false;
april_tag_detection::GetAprilTag april_tag_srv;
map::Map ieee_map(1);
double tof_data[3];
uint8_t landing_pad = -1;

bool failed_finding_beacon_mask = false;
/*
void botCallBack(const geometry_msgs::Pose& pose){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0));
        transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,  pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_map", "sensor"));
}*/

// Function to send a command to the Arduino
void sendCommand(unsigned int command) {
    if (!arduino_active) return;
    if (serial_port.isOpen()) {
        std::ostringstream command_stream;
        command_stream << "$" << command << "\n";
        serial_port.write(command_stream.str());
    } else {
        ROS_WARN("Serial port not open.");
    }
}

void findBeaconMask(bool& started, bool& done, map::checkPointData& cpData){
	static bool backUp = false;
	static ros::Time timer;
	static bool timer_started = false;
	if(!started && !done){
		started = true;
//		ieee_map.setMaxLinVel(4);
		ROS_INFO("Check Point: %d", cpData.id);
		ROS_INFO("Finding beacon mask...");
	}
	april_tag_srv.request.tag_id = "*";
	april_tag_srv.request.cam_id = "0";

	if (april_tag_client.call(april_tag_srv)){
		//ROS_INFO("Tag count: %d", april_tag_srv.response.tag_count);
		if (april_tag_srv.response.tag_count > 0){
			if (landing_pad == -1) landing_pad = april_tag_srv.response.tag_id[0];
			geometry_msgs::Pose tag_pose = april_tag_srv.response.poses[0];
			double x_error = TARGET_BEACON_Z - tag_pose.position.z;
			double y_error = tag_pose.position.x - TARGET_BEACON_X;
			ROS_INFO("X: %.2f, Y: %.2f", x_error, y_error);
			if (abs(x_error) > TARGET_BEACON_THRESHOLD && abs(y_error) > TARGET_BEACON_THRESHOLD){
				cpData.destination_pose.position.x = cpData.current_pose.position.x + x_error;
				cpData.destination_pose.position.y = cpData.current_pose.position.y + y_error;
				cpData.arrived = false;
			}
			else{
				//cpData.destination_pose.position.x = cpData.current_pose.position.x;
				//cpData.destination_pose.position.y = cpData.current_pose.position.y;
				//cpData.arrived = false;
				done = true;
				tof_active = false;
				ieee_map.resetMaxLinVel();
				ROS_INFO("Reached beacon mask!");
			}
    	} else {
			ROS_WARN("Cannot find beacon mask!");
			if(!backUp){
				timer_started = true;
				timer = ros::Time::now();
				backUp = true;
				cpData.destination_pose.position.x = cpData.current_pose.position.x + 3;
				cpData.arrived = false;
			}
			else if (timer_started && (ros::Time::now() - timer).toSec() >= 5){
				failed_finding_beacon_mask = true;
				done = true;
				ROS_ERROR("Cannot request AprilTag");
			}
		}
	}
}

void southAprilTagAlignment(bool& started, bool& done, map::checkPointData& cpData){
	static bool backUp = false;
	static ros::Time timer;
	static bool timer_started = false;
	if(!started && !done){
		started = true;
		april_tag_srv.request.tag_id = "*";
		april_tag_srv.request.cam_id = "0";
	}

	if (april_tag_client.call(april_tag_srv)){
		//ROS_INFO("Tag count: %d", april_tag_srv.response.tag_count);
		if (april_tag_srv.response.tag_count > 0){
			ROS_INFO("Tag #%d found!", april_tag_srv.response.tag_id[0]);
			geometry_msgs::Pose tag_pose = april_tag_srv.response.poses[0];
			mecanum_ik::adjustPositionSrv pos_srv;
			pos_srv.request.position.x = tag_pose.position.x + SOUTH_TAG_POS_X;
			pos_srv.request.position.y = tag_pose.position.z + SOUTH_TAG_POS_Y + 6;
			pos_srv.request.position.z = 0;
			if (mecanum_ik_adjust_pos_client.call(pos_srv)){
				done = true;
				ROS_INFO("X: %.2f, Y: %.2f", cpData.current_pose.position.x, cpData.current_pose.position.y);
			}
    	} else {
			ROS_WARN("Cannot find the south tag!");
			if(!backUp){
				timer_started = true;
				timer = ros::Time::now();
				backUp = true;
			}
			else if (timer_started && (ros::Time::now() - timer).toSec() >= 5){
				failed_finding_beacon_mask = true;
				done = true;
				ROS_ERROR("Cannot request AprilTag");
			}
		}
	}
}

void approachGEB_CSC(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if (!started && !done){
		timer = ros::Time::now();
		started = true;
	}

	geometry_msgs::Vector3 vel;
	vel.x = 0;
	vel.y = 0;
	vel.z = 0;

	if ((ros::Time::now() - timer).toSec() >= 3.0){
		ROS_WARN("Failed to detect Geo CSC!");
	}
	else if (digitalRead(GEO_CSC_SENSOR_PIN) == HIGH){
		vel.x = -0.5;
	}
	else {
		ROS_INFO("Reached Geo CSC");
		done = true;
	}

	ieee_map.manualSpeedControl(vel);
}

void waitBeforeFindingBeacon(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		started = true;
		timer = ros::Time::now();
	}
	ros::Duration count_down = ros::Time::now() - timer;
	if (count_down.toSec() >= 1){
		done = true;
	}
}

void dest_action_wait(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		started = true;
		timer = ros::Time::now();
	}
	ros::Duration count_down = ros::Time::now() - timer;
	if (count_down.toSec() >= 1){
		done = true;
	}
}

void cmdCallback(const std_msgs::UInt16::ConstPtr& msg){
	sendCommand(msg->data);
}

void placeBeacon(bool& started, bool& done, map::checkPointData& cpData){
        static ros::Time timer;
		static bool tof_start = false;
		static bool tof_done = false;
		static bool timer_started = false;
		static uint8_t zig_zac_state = 0;
		static uint8_t zig_zac_loop = 0;
		static bool zig_zac_trigger = false;
		static bool tof_requested = false;
		/*if (failed_finding_beacon_mask){
			started = true;
			done = true;
			return;
		}*/
		if (!tof_requested){
			tof_sensor::Tof srv;
			srv.request.activate = true;
			tof_active = tof_client.call(srv);
			if (tof_active){
				timer = ros::Time::now();
				ieee_map.setMaxLinVel(3);
			}
			tof_requested = true;
		}
		if (tof_active){
			if (!tof_start){
				if ((ros::Time::now() - timer).toSec() >= 2.0) tof_start = true;
			}
			else {
				if (!tof_done){
					if (tof_new){
						tof_new = false;
						double tof_error = tof_data[TOF_BACK] * MM_TO_INCH - TOF_BEACON_DIST;
						ROS_INFO("tof error: %.2f", tof_error);
						if (abs(tof_error) >= TOF_ERROR_THRESHOLD){
							cpData.destination_pose.position.x = cpData.current_pose.position.x - tof_error;
							cpData.arrived = false;
						}
						else {
							tof_done = true;
							sendCommand(CMD_PLACE_BEACON);
							timer = ros::Time::now();
						}
					}
					//ROS_INFO("%.1f",tof_data[TOF_BACK]);
				} else if((ros::Time::now() - timer).toSec() >= 1.5){
					done = true;
					ieee_map.resetMaxLinVel();
				}
			}
			return;
		}

        if(!started && !done){
            started = true;
			cpData.destination_pose.position.x = cpData.current_pose.position.x - (TARGET_BEACON_Z - TRUE_TARGET_BEACON_Y);
			cpData.arrived = false;
			ieee_map.setMaxLinVel(3);
        }
		else if (started && !timer_started){
			ROS_INFO("Placing beacon...");
			sendCommand(CMD_PLACE_BEACON);
			timer_started = true;
			timer = ros::Time::now();
		}
		else if (!zig_zac_trigger) {
	        ros::Duration count_down = ros::Time::now() - timer;
	        if (count_down.toSec() >= 1.0){
				zig_zac_trigger = true;
//				ROS_INFO("Beacon placed!");
//                done = true;
	        }
		}
		else {
			switch (zig_zac_state++){
				case 1:
					cpData.destination_pose.position.y = cpData.current_pose.position.y + ZIG_ZAC_WIDTH / 2;
					cpData.arrived = false;
					break;
				case 2:
					cpData.destination_pose.position.x = cpData.current_pose.position.x + ZIG_ZAC_HEIGHT;
					cpData.arrived = false;
					break;
				case 3:
					cpData.destination_pose.position.y = cpData.current_pose.position.y - ZIG_ZAC_WIDTH;
					cpData.arrived = false;
					break;
				case 4:
					cpData.destination_pose.position.x = cpData.current_pose.position.x + ZIG_ZAC_HEIGHT;
					cpData.arrived = false;
					break;
				case 5:
					cpData.destination_pose.position.y = cpData.current_pose.position.y + ZIG_ZAC_WIDTH;
					cpData.arrived = false;
					if (++zig_zac_loop > ZIG_ZAC_LOOP) {
						done = true;
						ieee_map.resetMaxLinVel();
					}
					else zig_zac_state = 2;
					break;
				default: break;
			}
		}
}

void releaseBeacon(bool& started, bool& done, map::checkPointData& cpData){
	if (!started && !done){
		started = true;
		cpData.destination_pose.position.x = cpData.current_pose.position.x + 2;
		cpData.arrived = false;
	} else {
		sendCommand(CMD_RESET_BEACON);
		done = true;
	}
}

void resetBeacon(bool& started, bool& done, map::checkPointData& cpData){
    if(!started && !done){
		ROS_INFO("Check Point: %d", cpData.id);
		cpData.destination_pose.position.x = cpData.current_pose.position.x + 3;
		cpData.arrived = false;
		started = true;
    }
	else {
		sendCommand(CMD_RESET_BEACON);
		done = true;
	}
}
/*
void resetBeacon(bool& started, bool& done, map::checkPointData& cpData){
        sendCommand(CMD_RESET_BEACON);
		started = true;
        done = true;
}*/

void deployGathering(bool& started, bool& done, map::checkPointData& cpData){
        static ros::Time timer;
        if(!started && !done){
                started = true;
                timer = ros::Time::now();
                sendCommand(CMD_DEPLOY_GATHERING);
        }
        ros::Duration count_down = ros::Time::now() - timer;
        if (count_down.toSec() >= 1.0){
				done = true;
        }
}

// function to start gathering and sorting. These should run together and stop //together

void gatherSortStart(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		// code for the first call
		started = true;
		timer = ros::Time::now();

		// call to start each motor / function

		// sending commands to start the gathering system
		sendCommand(CMD_GATHER_IN);
		sendCommand(CMD_LEFT_BELT_IN);
		sendCommand(CMD_RIGHT_BELT_IN);
		sendCommand(CMD_MID_BELT_IN);
		
		//sending commands to start the sort system
		sendCommand(CMD_RUN_SORTER);
		sendCommand(CMD_GEO_PUSH);
		sendCommand(CMD_NEB_PUSH);
	}
	// code during call
	ros::Duration count_down = ros::Time::now() - timer;
	if(count_down.toSec() >= 2.0){
		// stops the function after given time interval
		done = true;
		sendCommand(CMD_RESET_BEACON);
	}
}

void gatherStart(bool& started, bool& done, map::checkPointData& cpData){
        static ros::Time timer;
        if(!started && !done){
                // code for the first call
                started = true;
                timer = ros::Time::now();

                // call to start each motor / function

                // sending commands to start the gathering system
                sendCommand(CMD_GATHER_IN);
        }
        // code during call
        ros::Duration count_down = ros::Time::now() - timer;
        if(count_down.toSec() >= 0){
                // stops the function after given time interval
                done = true;
        }
}

// function to stop the gathering and sorting

void sortStop(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		// code for the first call
		started = true;
		timer = ros::Time::now();
	}
	// code during call
	ros::Duration count_down = ros::Time::now() - timer;
	if(count_down.toSec() >= 10.0){
		// stops the function after given time interval
		done = true;

		// call to start each motor / function

		// sending commands to start the gathering system
		sendCommand(CMD_LEFT_BELT_STOP);
		sendCommand(CMD_RIGHT_BELT_STOP);
		sendCommand(CMD_MID_BELT_STOP);
		
		//sending commands to start the sort system
		sendCommand(CMD_STOP_SORTER); 
	}
}


void gatherStop(bool& started, bool& done, map::checkPointData& cpData){
        static ros::Time timer;
        if(!started && !done){
                // code for the first call
                started = true;
                timer = ros::Time::now();
        }
        // code during call
        ros::Duration count_down = ros::Time::now() - timer;
        if(count_down.toSec() >= 1.0){
                // stops the function after given time interval
                done = true;

                // call to start each motor / function

                // sending commands to start the gathering system
                sendCommand(CMD_GATHER_STOP);
        }
}

void containerGrabNeb(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		// code for the first call
		started = true;
		timer = ros::Time::now();

		// call to start each motor / function

		// sending commands to dump nebulite
		sendCommand(CMD_NEB_GRAB);
		sendCommand(CMD_NEB_DUMP);
		
		
	}
	// code during call
	ros::Duration count_down = ros::Time::now() - timer;
	if(count_down.toSec() >= 0){
		// stops the function after given time interval
		done = true;
	}
}

void containerGrabGeo(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		started = true;
		timer = ros::Time::now();
		sendCommand(CMD_GEO_GRAB);
	}
	ros::Duration count_down = ros::Time::now() - timer;
	if(count_down.toSec() >= 2){
		done = true;
	}
}


void containerUngrabGeo(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		// code for the first call
		started = true;
		timer = ros::Time::now();
		sendCommand(CMD_GEO_UNGRAB);
		sendCommand(CMD_GEO_PUSH);
	}
	// code during call
	ros::Duration count_down = ros::Time::now() - timer;
	if(count_down.toSec() >= 2.0){
		// stops the function after given time interval
		done = true;
	}
}


void letGoGeo(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		// code for the first call
		started = true;
		timer = ros::Time::now();

		// call to start each motor / function

		// sending commands to dump geodinium
		sendCommand(CMD_GEO_UNGRAB);
		sendCommand(CMD_GEO_PUSH);
		
		
	}
	// code during call
	ros::Duration count_down = ros::Time::now() - timer;
	if(count_down.toSec() >= 2.0){
		// stops the function after given time interval
		done = true;
	}
}

void letGoNeb(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		// code for the first call
		started = true;
		timer = ros::Time::now();

		// call to start each motor / function

		// sending commands to dump geodiniumresetBeacon
		sendCommand(CMD_NEB_UNGRAB);
		sendCommand(CMD_NEB_PUSH);
	}
	// code during call
	ros::Duration count_down = ros::Time::now() - timer;
	if(count_down.toSec() >= 2.0){
		// stops the function after given time interval
		done = true;
	}
}

void lightOn(bool& started, bool& done, map::checkPointData& cpData){
	static ros::Time timer;
	if(!started && !done){
		// code for the first call
		started = true;
		timer = ros::Time::now();

		// call to start each motor / function

		// sending commands to dump geodinium
		sendCommand(CMD_FLOOD_LIGHT_ON);
	}
	// code during call
	ros::Duration count_down = ros::Time::now() - timer;
	if(count_down.toSec() >= 2.0){
		// stops the function after given time interval
		done = true;
	}
}

void reset(){
	ieee_map.run_path = false;
	std_srvs::Empty srv;
	if (mecanum_ik_client.call(srv)) {
        	ROS_INFO_STREAM("Wheel is shutdown.");
	} else {
        	ROS_ERROR("Failed to shutdown wheel.");
    	}

	for (int i=20; i<30; i++){
		sendCommand(i);
		ros::Duration(0.1).sleep();
	}
	sendCommand(40);
	serial_port.close();
}

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
		ros::Duration(1.5).sleep();
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

void tofCallback(const std_msgs::Float64MultiArray::ConstPtr& tof_msg){
	tof_data[0] = tof_msg->data[0];
	tof_data[1] = tof_msg->data[1];
	tof_data[2] = tof_msg->data[2];
	tof_new = true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "IEEE_map");
	ros::NodeHandle nh;
	ros::Rate rate(60);

	ros::Subscriber cmd_sub = nh.subscribe("subsystem_command", 1, cmdCallback);
	ros::Subscriber tof_sub = nh.subscribe("tof_sensor", 10, tofCallback);
	bot_vel_sub = nh.advertise<geometry_msgs::Vector3>("Bot_Velocities", 1);
	april_tag_client = nh.serviceClient<april_tag_detection::GetAprilTag>("get_april_tag");
	mecanum_ik_client = nh.serviceClient<april_tag_detection::GetAprilTag>("shutdown_wheel");
	tof_client = nh.serviceClient<tof_sensor::Tof>("tof_srv");
	mecanum_ik_adjust_pos_client = nh.serviceClient<mecanum_ik::adjustPositionSrv>("mecanum_ik_srv");

	ros::Duration srvTimeout(3.0); // 5 seconds timeout
    if (april_tag_client.waitForExistence(srvTimeout)) {
        ROS_INFO("April tag detection ready!");
    } else {
        ROS_ERROR("April tag detection is not available!");
    }
    if (mecanum_ik_client.waitForExistence(srvTimeout)) {
        ROS_INFO("Wheel controller is ready!");
    } else {
        ROS_ERROR("Failed to start wheel controller!");
    }

	//mecanum_ik_client = nh.serviceClient<mecanum_ik::EmptySrv>("shutdown_wheel");
	//ros::service::waitForService("shutdown_wheel");

	serial_port.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
	serial_port.setTimeout(timeout);

    serial_port.setPort("/dev/USB_SYS");
    serial_port.open();
	arduino_active = false;
	if (!serial_port.isOpen()) {
		ROS_ERROR("Cannot open subsystem (Arduino Mega)!");
		return 0;
    }
	arduino_active = true;

	ieee_map.init(&nh, "1", true);

	geometry_msgs::Vector3 box_size;
	box_size.x = 4;
	box_size.y = 4;
	box_size.z = 1;

	box_size.x = 2.5;
	box_size.y = 2.5;
	ieee_map.createContainersController(box_size, 3.5);

	unsigned int pad_num = 0;

	// approaching GEO CSC test
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25, 9.5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(NO_MOVE, NO_MOVE, 0), M_PI_2, approachGEB_CSC, map::Map::doNothing);


/*
	// South Tag alignment test
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25, 6.5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(39.5, 6.5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(39.5, 11.5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(45.7, 11.5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(40, 15, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(44, 15, 0), M_PI, map::Map::doNothing, southAprilTagAlignment);

	// turn back to GEO CSC
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(NO_MOVE, NO_MOVE, 0), -M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(42, 6, 0), -M_PI_2, map::Map::doNothing, map::Map::doNothing);
*/

	// Auto Speed Test
/*	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25-20, 6, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25, 6 + 10, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25 - 20, 6 + 10, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25 - 20, 6 + 5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25, 6 + 5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25, 6, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
*/

/*
	// back up and deploy gathering
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25 - 15, 6, 0), M_PI_2,  map::Map::doNothing, deployGathering);

	// approaching beacon mask, red
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(31.25 - 15, 22.4, 0), M_PI_2, map::Map::doNothing, waitBeforeFindingBeacon);

	// find beacon mask
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(NO_MOVE, NO_MOVE, 0), M_PI_2, map::Map::doNothing, findBeaconMask);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(NO_MOVE, NO_MOVE, 0), M_PI_2, map::Map::doNothing, placeBeacon);

	// release beacon
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(NO_MOVE, NO_MOVE, 0), M_PI_2, map::Map::doNothing, releaseBeacon);

	// Auto home
//	ieee_map.addCheckPoint(pad_num++, tf::Vector3(30.5, NO_MOVE, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
//	ieee_map.addCheckPoint(pad_num++, tf::Vector3(30.5, 6, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);

//	ieee_map.addCheckPoint(pad_num++, tf::Vector3(8.6, 22.6, 0), M_PI, map::Map::doNothing, map::Map::doNothing);

	// Turn toward the N wall to gather ore
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(15, 22.5, 0), M_PI, gatherSortStart, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(15, 36, 0), M_PI, map::Map::doNothing, dest_action_wait);

	ieee_map.addCheckPoint(pad_num++, tf::Vector3(15, 22.5, 0), M_PI, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(15, 22.5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);


	// forward into the cave
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(53, 22.5, 0), M_PI_2, gatherSortStart, map::Map::doNothing);
//	ieee_map.addCheckPoint(pad_num++, tf::Vector3(53, 22.5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);

	// back up out of the cave
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(44, 22.5, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);

	// turn to N
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(44, 22.5, 0), M_PI, map::Map::doNothing, map::Map::doNothing);

	// go close to the cave
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(50, 22.5, 0), M_PI, map::Map::doNothing, map::Map::doNothing);

	// into the N wall
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(50, 35.5, 0), M_PI, map::Map::doNothing, dest_action_wait);

	// push ore to the west
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(46, 35.5, 0), M_PI, map::Map::doNothing, map::Map::doNothing);

	// back up
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(46, 18, 0), M_PI, map::Map::doNothing, map::Map::doNothing);

	// move west
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(36, 18, 0), M_PI, map::Map::doNothing, map::Map::doNothing);

	// into the N wall
//	ieee_map.addCheckPoint(pad_num++, tf::Vector3(35.5, 36.5, 0), M_PI, map::Map::doNothing, gatherStop);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(36, 36, 0), M_PI, map::Map::doNothing, gatherStop);

	// push Neb cont to landing pad
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(13.5, 36, 0), M_PI, map::Map::doNothing, map::Map::doNothing);

	// rotate to W
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(19, 34.5, 0), M_PI, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(19, 34.5, 0), -M_PI_2, map::Map::doNothing, map::Map::doNothing);


	// adjust before gather
//	ieee_map.addCheckPoint(pad_num++, tf::Vector3(19, 35.25, 0), -M_PI_2, map::Map::doNothing, map::Map::doNothing);

	// gather to W wall
//	ieee_map.addCheckPoint(pad_num++, tf::Vector3(10, 35.25, 0), -M_PI_2, gatherStart, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(10.3, 35.25, 0), -M_PI_2, gatherStart, map::Map::doNothing);

	// rotate to S wall
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(14, 35.25, 0), 0, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(6.2, 35.5, 0), 0, map::Map::doNothing, map::Map::doNothing);

	// move to S wall
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(6.2, 10.75, 0), 0, map::Map::doNothing, map::Map::doNothing);

	// rotate to E
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(9, 14, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(9, 6, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);

	ieee_map.addCheckPoint(pad_num++, tf::Vector3(40.5, 6, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);

	// rotate to W
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(40.5, 8, 0), M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(40.5, 8, 0), M_PI, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(40.5, 8, 0), -M_PI_2, map::Map::doNothing, map::Map::doNothing);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(40.5, 6, 0), -M_PI_2, map::Map::doNothing, map::Map::doNothing);

	// approaching G container
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(46.8, 5.5, 0), -M_PI_2, map::Map::doNothing, containerGrabGeo);
	ieee_map.addCheckPoint(pad_num++, tf::Vector3(14, 9, 0), -M_PI_2, map::Map::doNothing, map::Map::doNothing);
*/
	ieee_map.publishField();

	ieee_map.reset = reset;

	wiringPiSetupGpio();
	pullUpDnControl(LIGHT_DETECTOR_PIN, INPUT);
	pullUpDnControl(GEO_CSC_SENSOR_PIN, INPUT);
	pullUpDnControl(NEB_CSC_SENSOR_PIN, INPUT);

	ros::Time game_time;
	ros::Duration(1).sleep();
	ROS_INFO("Bot Ready!");

	while(ros::ok()){
		ROS_INFO("GEO SENSOR: %d, NEB_SENSOR: %d", digitalRead(GEO_CSC_SENSOR_PIN), digitalRead(NEB_CSC_SENSOR_PIN));
		ros::spinOnce();
		rate.sleep();
	}

	while(ros::ok()){
		if(!ieee_map.run_path) {
			if(digitalRead(LIGHT_DETECTOR_PIN) == LOW){
				ieee_map.run_path = true;
				game_time = ros::Time::now();
//				ROS_INFO("Start!");
			}
		}
		else if((ros::Time::now() - game_time).toSec() > 180 || !ieee_map.run_path){
			reset();
			return 1;
			break;
		} else ieee_map.followPath();

		ros::spinOnce();
		rate.sleep();
	}

	reset();
	return 0;
}
