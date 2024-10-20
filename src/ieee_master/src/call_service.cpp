#include <ros/ros.h>
#include <keyboard/Key.h>
#include <std_srvs/Empty.h>
#include <apriltag_ros/common_functions.h>
#include <apriltag_ros/AnalyzeSingleImage.h>

ros::ServiceClient img_save_client;
ros::ServiceClient apriltag_client;
apriltag_ros::AnalyzeSingleImage apriltag_service;

const char* img_save_srv_name = "image_saver/save";

void callbackKeyboard(const keyboard::Key& key_msg){
	if (key_msg.code == keyboard::Key::KEY_c){
		if(ros::service::exists(img_save_srv_name, true)){
			std_srvs::Empty srv;
			if(img_save_client.call(srv)){
				ros::Duration(0.5).sleep();
				if(apriltag_client.call(apriltag_service))
				{
    					// use parameter run_quielty=false in order to have the service
 		 	 	 	// print out the tag position and orientation
    					if (apriltag_service.response.tag_detections.detections.size() == 0){
      						ROS_WARN_STREAM("No detected tags!");
    					}
  				}
  				else{
    					ROS_ERROR("Failed to call service single_image_tag_detection");
  				}
			}
			else{
				ROS_ERROR("Failed to call service %s", img_save_srv_name);
                        }

		}
	}
}

bool getRosParameter (ros::NodeHandle& pnh, std::string name, double& param)
{
  // Write parameter "name" from ROS Parameter Server into param
  // Return true if successful, false otherwise
  if (pnh.hasParam(name.c_str()))
  {
    pnh.getParam(name.c_str(), param);
    ROS_INFO_STREAM("Set camera " << name.c_str() << " = " << param);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Could not find " << name.c_str() << " parameter!");
    return false;
  }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "capture_apriltag");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	ros::Subscriber sub = nh.subscribe("keyboard/keydown", 1, callbackKeyboard);

	img_save_client = nh.serviceClient<std_srvs::Empty>(img_save_srv_name);

	apriltag_client = nh.serviceClient<apriltag_ros::AnalyzeSingleImage>("single_image_tag_detection");

	// Get the request parameters
	apriltag_service.request.full_path_where_to_get_image = apriltag_ros::getAprilTagOption<std::string>( pnh, "image_load_path", "");
	if (apriltag_service.request.full_path_where_to_get_image.empty())
	{
		return 1;
	}
	apriltag_service.request.full_path_where_to_save_image =
	apriltag_ros::getAprilTagOption<std::string>(pnh, "image_save_path", "");
  	if (apriltag_service.request.full_path_where_to_save_image.empty())
  	{
    		return 1;
  	}

	// Replicate sensors_msgs/CameraInfo message (must be up-to-date with the
  	// analyzed image!)  
  	apriltag_service.request.camera_info.distortion_model = "plumb_bob";
  	double fx, fy, cx, cy;
  	if (!getRosParameter(pnh, "fx", fx)) return 1;
  	if (!getRosParameter(pnh, "fy", fy)) return 1;
  	if (!getRosParameter(pnh, "cx", cx)) return 1;
  	if (!getRosParameter(pnh, "cy", cy)) return 1;
  	// Intrinsic camera matrix for the raw (distorted) images
  	apriltag_service.request.camera_info.K[0] = fx;
  	apriltag_service.request.camera_info.K[2] = cx;
 	apriltag_service.request.camera_info.K[4] = fy;
  	apriltag_service.request.camera_info.K[5] = cy;
  	apriltag_service.request.camera_info.K[8] = 1.0;
  	apriltag_service.request.camera_info.P[0] = fx;
  	apriltag_service.request.camera_info.P[2] = cx;
  	apriltag_service.request.camera_info.P[5] = fy;
  	apriltag_service.request.camera_info.P[6] = cy;
  	apriltag_service.request.camera_info.P[10] = 1.0;

	ros::spin();
	return 0;
}
