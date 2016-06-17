#include <ros/ros.h>
#include <vector>
#include <iterator>
#include <string>

#include <dji_sdk/dji_sdk.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Geometry> 


ros::Subscriber velocity_x_sub;
ros::Subscriber velocity_y_sub;

ros::ServiceClient velocity_control_service;
ros::ServiceClient sdk_permission_control_service;

double velocity_control_effort_x;
double velocity_control_effort_y;


void velocityControlEffortXCallback(std_msgs::Float64 velocity_control_effort_x_msg)
{
	velocity_control_effort_x = velocity_control_effort_x_msg.data;
}

void velocityControlEffortYCallback(std_msgs::Float64 velocity_control_effort_y_msg)
{
	velocity_control_effort_y = velocity_control_effort_y_msg.data;
}


//ros::ServiceClient velocity_control_service;
//velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("dji_sdk/velocity_control");
//return velocity_control_service.call(velocity_control) && velocity_control.response.result;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "landing_controller");

	ros::NodeHandle nh;

	velocity_x_sub = nh.subscribe("/teamhku/velocity_control_effort_x", 1000, velocityControlEffortXCallback);
	velocity_y_sub = nh.subscribe("/teamhku/velocity_control_effort_y", 1000, velocityControlEffortYCallback);

	velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("dji_sdk/velocity_control");
	sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");

	dji_sdk::VelocityControl velocity_control;

	int frame = 0;
	int yaw_rate = 0;

	dji_sdk::SDKPermissionControl sdk_permission_control;
	sdk_permission_control.request.control_enable = 1;

	while(!(sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result))
	{
		ROS_ERROR("request control failed!");
	}


	while(ros::ok())
	{
		ros::spinOnce();

		velocity_control.request.frame = frame;
		velocity_control.request.vx = velocity_control_effort_x;
		velocity_control.request.vy = velocity_control_effort_y;
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