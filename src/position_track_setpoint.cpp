#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <dji_sdk/Gimbal.h>

#include <apriltags/AprilTagDetections.h>


#include <cmath>
#include <Eigen/Geometry> 


ros::Subscriber apriltags_pos_sub;
ros::Subscriber gimbal_ori_sub;

ros::Publisher setpoint_x_pub;
ros::Publisher setpoint_y_pub;

std_msgs::Float64 setpoint_yaw;
std_msgs::Float64 setpoint_pitch;

double gimbal_roll;
double gimbal_yaw;
double gimbal_pitch;

double tag_x;
double tag_y;
double tag_z;

std::string tag_detection_topic;

// void apriltagsPositionCallback(const geometry_msgs::PoseArray::ConstPtr& apriltag_pos_msg)
// {
//   if(std::begin(apriltag_pos_msg->poses) == std::end(apriltag_pos_msg->poses))
//   {
//     return;
//   }
//   else
//   {
//     tag_x = apriltag_pos_msg->poses[0].position.x;
//     tag_y = apriltag_pos_msg->poses[0].position.y;
//     tag_z = apriltag_pos_msg->poses[0].position.z;


//   }
  
// }


void apriltagsPositionCallback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    return;
  }
  else
  {
    tag_x = apriltag_pos_msg->detections[0].pose.position.x;
    tag_y = apriltag_pos_msg->detections[0].pose.position.y;
    tag_z = apriltag_pos_msg->detections[0].pose.position.z;

  }
}

void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
  gimbal_roll = gimbal_ori_msg->roll * M_PI / 180;
  gimbal_yaw = gimbal_ori_msg->yaw * M_PI / 180;
  gimbal_pitch = gimbal_ori_msg->pitch * M_PI / 180;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_track_setpoint_node");
  ROS_INFO("Starting position track setpoint publisher");
  ros::NodeHandle nh;

  while (ros::Time(0) == ros::Time::now())
  {
    ROS_INFO("Setpoint_node spinning waiting for time to become non-zero");
    sleep(1);
  }

  nh.param<std::string>("/position_track_setpoint/tag_detection_topic", tag_detection_topic, "/apriltags_ros/tag_detections_pose");
  ROS_INFO("Listening to apriltag detection topic: %s", tag_detection_topic.c_str());

  setpoint_x_pub = nh.advertise<std_msgs::Float64>("/teamhku/position_track/setpoint_x", 1);
  setpoint_y_pub = nh.advertise<std_msgs::Float64>("/teamhku/position_track/setpoint_y", 1);

  // apriltags_pos_sub = nh.subscribe(tag_detection_topic, 1000, apriltagsPositionCallback);
  apriltags_pos_sub = nh.subscribe(tag_detection_topic, 1000, apriltagsPositionCallback);

  gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);


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

  std_msgs::Float64 setpoint_x_msg;
  std_msgs::Float64 setpoint_y_msg;

  ros::Rate loop_rate(200); 

  while (ros::ok())
  {
    ros::spinOnce();

    yawAngle = Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ());
    pitchAngle = Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitY());
    rollAngle = Eigen::AngleAxisd(gimbal_roll, Eigen::Vector3d::UnitX());

    q = rollAngle * pitchAngle * yawAngle;

    T_c_f = q.matrix() * T_c_g;

    P_c << tag_x, tag_y, tag_z;

    P_f = T_c_f * P_c;

    setpoint_x_msg.data = P_f(0);
    setpoint_y_msg.data = P_f(1);

    setpoint_x_pub.publish(setpoint_x_msg);
    setpoint_y_pub.publish(setpoint_y_msg);

    loop_rate.sleep();
  }
}
