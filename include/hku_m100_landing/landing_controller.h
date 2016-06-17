#ifndef LANDING_CONTROLLER_H
#define LANDING_CONTROLLER_H

#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

namespace teamhku{

class LandingController{
public:
	LandingController();
	~LandingController();

private:
	ros::NodeHandle nh_;

	ros::Subscriber apriltag_pos_sub_;

	ros::ServiceClient velocity_control_service_;

};

}


#endif