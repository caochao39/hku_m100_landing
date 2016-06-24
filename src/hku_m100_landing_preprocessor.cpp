#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <vector>
#include <iterator>
#include <string>

#include <dji_sdk/Gimbal.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Geometry> 
#include <cmath>



// std::vector<geometry_msgs::PoseArray> apriltags_arry;
// geometry_msgs::Pose* apriltag_poses;

double gimbal_roll;
double gimbal_yaw;
double gimbal_pitch;

const int landing_tag_id = 6;
bool landing_tag_found = false;

double tag_x;
double tag_y;
double tag_z;

ros::Subscriber apriltags_pos_sub;
ros::Subscriber gimbal_ori_sub;

ros::Publisher tag_x_pub;
ros::Publisher tag_y_pub;

void apriltagsPositionCallback(const geometry_msgs::PoseArray::ConstPtr& apriltag_pos_msg)
{
	// apriltag_poses = apriltag_pos_msg->poses;
	if(std::begin(apriltag_pos_msg->poses) == std::end(apriltag_pos_msg->poses))
	{
		return;
	}
	tag_x = apriltag_pos_msg->poses[0].position.x;
	tag_y = apriltag_pos_msg->poses[0].position.y;
	tag_z = apriltag_pos_msg->poses[0].position.z;

	landing_tag_found = true;

}

void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
	gimbal_roll = gimbal_ori_msg->roll * M_PI / 180;
	gimbal_yaw = gimbal_ori_msg->yaw * M_PI / 180;
	gimbal_pitch = gimbal_ori_msg->pitch * M_PI / 180;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "landing_preprocessor");

	ros::NodeHandle nh;

	apriltags_pos_sub = nh.subscribe("/dji_sdk/class_36h11/tag_detections_pose", 1000, apriltagsPositionCallback);
	gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);

	tag_x_pub = nh.advertise<std_msgs::Float64>("/teamhku/tag_x", 1000);
	tag_y_pub = nh.advertise<std_msgs::Float64>("/teamhku/tag_y", 1000);


	Eigen::Matrix3d T_c_g;
	T_c_g << 0, 0, 1,
			 1, 0, 0,
			 0, 1, 0;

	Eigen::AngleAxisd yawAngle;
	Eigen::AngleAxisd pitchAngle;
	Eigen::AngleAxisd rollAngle;

	Eigen::Quaternion<double> q;

	Eigen::Matrix3d T_c_f;

	Eigen::Vector3d P_c;

	Eigen::Vector3d P_f;

	std_msgs::Float64 tag_x_msg;
	std_msgs::Float64 tag_y_msg;

	double gimbal_yaw_tmp = (-1) * 3.14159 / 180;
	double gimbal_pitch_tmp = (-90) * 3.14159 / 180;
	double gimbal_roll_tmp = 0;

	ros::Rate spin_rate(100);// to be modified

	while(ros::ok())
	{
		ros::spinOnce();

		//control
		//for gazebo simulation
		yawAngle = Eigen::AngleAxisd(gimbal_yaw_tmp, Eigen::Vector3d::UnitZ());
		pitchAngle = Eigen::AngleAxisd(gimbal_pitch_tmp, Eigen::Vector3d::UnitY());
		rollAngle = Eigen::AngleAxisd(gimbal_roll_tmp, Eigen::Vector3d::UnitX());

		//for real robot
		// yawAngle = Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ());
		// pitchAngle = Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitY());
		// rollAngle = Eigen::AngleAxisd(gimbal_roll, Eigen::Vector3d::UnitX());

		q = rollAngle * pitchAngle * yawAngle;

		T_c_f = q.matrix() * T_c_g;

		P_c << tag_x, tag_y, tag_z;

		P_f = T_c_f * P_c;

		tag_x_msg.data = P_f(0);
		tag_y_msg.data = P_f(1);

		tag_x_pub.publish(tag_x_msg);
		tag_y_pub.publish(tag_y_msg);

		spin_rate.sleep();
	}

	return 0;
}