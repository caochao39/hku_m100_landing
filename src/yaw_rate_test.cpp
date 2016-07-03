#include <ros/ros.h>
#include <string>

#include <dji_sdk/dji_sdk.h>

#include <cmath>
#include <iostream>

ros::ServiceClient velocity_control_service;
ros::ServiceClient sdk_permission_control_service;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_track_controller");
	ros::NodeHandle nh;
	
	velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("dji_sdk/velocity_control");
	sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");

	dji_sdk::VelocityControl velocity_control;

	int frame = 0;
	double yaw_rate = 10;

	dji_sdk::SDKPermissionControl sdk_permission_control;
	sdk_permission_control.request.control_enable = 1;
	bool control_requested = false;

	while(!(sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result))
	{
		ROS_ERROR("request control failed!");
	}

	while(ros::ok())
	{
		ros::spinOnce();

		velocity_control.request.frame = frame;
		velocity_control.request.vx = 0;
		velocity_control.request.vy = 0;
		velocity_control.request.vz = 0;
		velocity_control.request.yawRate = yaw_rate;

		if(!(velocity_control_service.call(velocity_control) && velocity_control.response.result))
		{
			ROS_ERROR("velocity control failed!");
		}
	}

	sdk_permission_control.request.control_enable = 0;
	sdk_permission_control_service.call(sdk_permission_control);

}