#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
// #include <apriltags_ros/AprilTagDetection.h>
// #include <apriltags_ros/AprilTagDetectionArray.h>
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/AttitudeQuaternion.h>

#include <tag.h>

#include <cmath>
#include <Eigen/Geometry> 

#include <vector>
#include <iterator>
#include <algorithm>

// #define APRILTAGS_ROS
//#define APRILTAGS

ros::Subscriber apriltags_36h11_sub;
ros::Subscriber apriltags_16h5_sub;
ros::Subscriber gimbal_ori_sub;
ros::Subscriber local_position_sub;
ros::Subscriber landing_enable_sub;
ros::Subscriber attitude_quaternion_sub;

ros::Publisher setpoint_x_pub;
ros::Publisher setpoint_y_pub;
ros::Publisher setpoint_yaw_pub;
ros::Publisher landing_condition_met_pub;
ros::Publisher relanding_condition_met_pub;
ros::Publisher yaw_state_pub;


double gimbal_roll;
double gimbal_yaw;
double gimbal_pitch;

double local_x;
double local_y;
double local_z;

double error_yaw;

double heading_q0;
double heading_q1;
double heading_q2;
double heading_q3;

Eigen::Quaternion<double> drone_heading;
double yaw_state = 0;

double setpoint_x = 0;
double setpoint_y = 0;
double setpoint_z = 0;
double setpoint_yaw = 0;

const int tag_16h5_num = 7;
double landing_threshold = 0.1;
double reland_horizontal_threshold = 0.3;
double reland_height_min_threshold = 0.2;
double reland_height_max_threshold = 2.5;
double absolute_reland_height_min_threshold = 1.3;

bool relanding = false;


std_msgs::Bool landing_condition_met_msg;
std_msgs::Bool relanding_condition_met_msg;

Tag *tag_36h11_6;

std::vector<Tag *> tags_16h5;
std::vector<int> found_tag_id;
std::vector<double> yaw_errors;

std::string tag_16h5_detection_topic;
std::string tag_36h11_detection_topic;

bool found_36h11 = false;
bool found_16h5 = false;

bool landing_enabled = false;

#ifdef APRILTAGS
#include <apriltags/AprilTagDetections.h>

void apriltags36h11Callback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_36h11 = false;
    return;
  }
  else
  {
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id == 6)
      {
        tag_36h11_6->updateTagState((*it).pose);

      }
      
    }

    found_36h11 = true;

  }
}


void apriltags16h5Callback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_16h5 = false;
    return;
  }
  else
  {
    found_tag_id.clear();
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id >= 0 && (*it).id <= 6)
      {
        tags_16h5[(*it).id]->updateTagState((*it).pose);
        found_tag_id.push_back((*it).id);
      }
      
    }
    for(int i = 0; i < tag_16h5_num; i++)
    {
      if(std::find(found_tag_id.begin(), found_tag_id.end(), i) == found_tag_id.end())
      {
        tags_16h5[i]->setMissing();
      }
    }

    found_16h5 = true;
  }
}

#else

#include <apriltags_ros/AprilTagDetectionArray.h>

void apriltags36h11Callback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_36h11 = false;
    tag_36h11_6->setMissing();
    return;
  }
  else
  {
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id == 6)
      {
        tag_36h11_6->updateTagState((*it).pose.pose);
      }
      
    }

    found_36h11 = true;

  }
}


void apriltags16h5Callback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_16h5 = false;
    for(int i = 0; i < tag_16h5_num; i++)
    {
      tags_16h5[i]->setMissing();
    }
    return;
  }
  else
  {
    found_tag_id.clear();
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id >= 0 && (*it).id <= 6)
      {
        tags_16h5[(*it).id]->updateTagState((*it).pose.pose);
        found_tag_id.push_back((*it).id);
      }
      
    }
    for(int i = 0; i < tag_16h5_num; i++)
    {
      if(std::find(found_tag_id.begin(), found_tag_id.end(), i) == found_tag_id.end())
      {
        tags_16h5[i]->setMissing();
      }
    }

    found_16h5 = true;
  }
}

#endif

void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
  gimbal_roll = gimbal_ori_msg->roll * M_PI / 180;
  gimbal_yaw = gimbal_ori_msg->yaw * M_PI / 180;
  gimbal_pitch = gimbal_ori_msg->pitch * M_PI / 180;
}

void localPositionCallback(const dji_sdk::LocalPosition::ConstPtr& local_position_msg)
{
  local_x = local_position_msg->x;
  local_y = local_position_msg->y;
  local_z = local_position_msg->z;
}

void attitudeQuaternionCallback(const dji_sdk::AttitudeQuaternion::ConstPtr& attitude_quaternion_msg)
{
  heading_q0 = attitude_quaternion_msg->q0;
  heading_q1 = attitude_quaternion_msg->q1;
  heading_q2 = attitude_quaternion_msg->q2;
  heading_q3 = attitude_quaternion_msg->q3;

  drone_heading = Eigen::Quaternion<double>(heading_q0, heading_q1, heading_q2, heading_q3);
  // yaw_state = drone_heading.toRotationMatrix().eulerAngles(0, 1, 2)(2) / M_PI * 180;
  yaw_state = atan2(2*(heading_q0 * heading_q3 + heading_q1 * heading_q2), 1 - 2 * (heading_q2 * heading_q2 + heading_q3 * heading_q3)) / M_PI * 180;
}



void landingEnableCallback(const std_msgs::Bool& landing_enable_msg)
{
  landing_enabled = landing_enable_msg.data;

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

  nh.param<std::string>("/position_track_setpoint/tag_36h11_detection_topic", tag_36h11_detection_topic, "/apriltags/36h11/detections");
  nh.param<std::string>("/position_track_setpoint/tag_16h5_detection_topic", tag_16h5_detection_topic, "/apriltags/16h5/detections");
  ROS_INFO("Listening to 36h11 apriltag detection topic: %s", tag_36h11_detection_topic.c_str());
  ROS_INFO("Listening to 16h5 apriltag detection topic: %s", tag_16h5_detection_topic.c_str());


  setpoint_x_pub = nh.advertise<std_msgs::Float64>("/teamhku/position_track/setpoint_x", 10);
  setpoint_y_pub = nh.advertise<std_msgs::Float64>("/teamhku/position_track/setpoint_y", 10);
  setpoint_yaw_pub = nh.advertise<std_msgs::Float64>("/teamhku/position_track/setpoint_yaw", 10);
  landing_condition_met_pub = nh.advertise<std_msgs::Bool>("/teamhku/position_track/landing_condition_met", 10);
  relanding_condition_met_pub = nh.advertise<std_msgs::Bool>("/teamhku/position_track/relanding_condition_met", 10);
  yaw_state_pub = nh.advertise<std_msgs::Float64>("/teamhku/position_track/yaw_state", 10);
  
  apriltags_36h11_sub = nh.subscribe(tag_36h11_detection_topic, 1000, apriltags36h11Callback);
  apriltags_16h5_sub = nh.subscribe(tag_16h5_detection_topic, 1000, apriltags16h5Callback);
  gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);
  local_position_sub = nh.subscribe("dji_sdk/local_position", 10, localPositionCallback);
  landing_enable_sub = nh.subscribe("/teamhku/position_track/landing_enable", 1, landingEnableCallback ); 
  attitude_quaternion_sub = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attitudeQuaternionCallback ); 

  for(int i = 0; i < tag_16h5_num; i++)
  {
    tags_16h5.push_back(new Tag());
  }
  
  tags_16h5[0]->setToLandingCenterTranslation(Eigen::Vector3d(-0.3,  0.0, 0.0));
  tags_16h5[1]->setToLandingCenterTranslation(Eigen::Vector3d(0.0,   0.0, 0.0));
  tags_16h5[2]->setToLandingCenterTranslation(Eigen::Vector3d(0.0,0.115, 0.0));
  tags_16h5[3]->setToLandingCenterTranslation(Eigen::Vector3d(0.3,   0.0, 0.0));
  tags_16h5[4]->setToLandingCenterTranslation(Eigen::Vector3d(0.0, -0.115, 0.0));
  tags_16h5[5]->setToLandingCenterTranslation(Eigen::Vector3d(0.0, 0.23, 0.0));
  tags_16h5[6]->setToLandingCenterTranslation(Eigen::Vector3d(0.0,  -0.23, 0.0));

  tag_36h11_6 = new Tag();
  tag_36h11_6->setToLandingCenterTranslation(Eigen::Vector3d(0.0,  0.485, 0.0));

  //camera to gimbal transformation
  Eigen::Matrix3d camera_to_gimbal_transformation;
    camera_to_gimbal_transformation << 0, 0, 1,
                                       1, 0, 0,
                                       0, 1, 0;

  Eigen::AngleAxisd yawAngle;
  Eigen::AngleAxisd pitchAngle;
  Eigen::AngleAxisd rollAngle;

  Eigen::Quaternion<double> q;

  //camera to drone transformation
  Eigen::Matrix3d camera_to_drone_transformation;

  std_msgs::Float64 setpoint_x_msg;
  std_msgs::Float64 setpoint_y_msg;
  std_msgs::Float64 setpoint_yaw_msg;
  std_msgs::Float64 yaw_state_msg;

  double yaw_sum = 0;
  double average_yaw_error = 0;
  double found_tag_count = 0;
  double tag_36h11_yaw_state;


  Eigen::Vector3d landing_center_position_sum(0, 0, 0);
  Eigen::Vector3d average_landing_center_position(0, 0, 0);
  Eigen::Vector3d landing_center_position;


  ros::Rate loop_rate(200); 

  ros::spinOnce();
  if(!found_16h5 && !found_36h11)
  {
    setpoint_x = local_x;
    setpoint_y = local_y;
  }

  while (ros::ok())
  {
    ros::spinOnce();

    if(found_16h5 || found_36h11)
    {

      yawAngle = Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ());
      pitchAngle = Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitY());
      rollAngle = Eigen::AngleAxisd(gimbal_roll, Eigen::Vector3d::UnitX());

      q = rollAngle * pitchAngle * yawAngle;

      camera_to_drone_transformation = q.matrix() * camera_to_gimbal_transformation;

      //get the averaging landing center position
      landing_center_position_sum = Eigen::Vector3d(0, 0, 0);
      found_tag_count = 0;  

    
      for(int i = 0; i < tag_16h5_num; i++)
        {
          if(tags_16h5[i]->isFound())
          {
            found_tag_count ++;
            tags_16h5[i]->calculateDroneFramePosition(camera_to_drone_transformation);
            tags_16h5[i]->calculateDroneFrameOrientation(camera_to_drone_transformation);
            landing_center_position = tags_16h5[i]->getLandingCenterPosition();
            landing_center_position_sum += landing_center_position;
            // std::cout << "id: " << i << " landing center: x: " << tags_16h5[i]->getLandingCenterPosition().x() << " y: " << tags_16h5[i]->getLandingCenterPosition().y() << " z: " << tags_16h5[i]->getLandingCenterPosition().z() << std::endl; 
          }
        }
        if(tag_36h11_6->isFound())
        {
          found_tag_count ++;
          tag_36h11_6->calculateDroneFramePosition(camera_to_drone_transformation);
          tag_36h11_6->calculateDroneFrameOrientation(camera_to_drone_transformation);
          landing_center_position_sum += tag_36h11_6->getLandingCenterPosition();
          // std::cout << "36h11" <<" landing center: x: " << tag_36h11_6->getLandingCenterPosition().x() << " y: " << tag_36h11_6->getLandingCenterPosition().y() << " z: " << tag_36h11_6->getLandingCenterPosition().z() << std::endl; 

        }
        // std::cout << std::endl;
        average_landing_center_position = landing_center_position_sum / found_tag_count;

      //get the averaging landing center yaw error
      yaw_sum = 0;
      yaw_errors.clear();
      for(int i = 0; i < tag_16h5_num; i++)
      {
        if(tags_16h5[i]->isFound())
          {
            yaw_errors.push_back(tags_16h5[i]->getYawError());
            // std::cout << "16h5 tags id: " << i << " yaw error: " <<  tags_16h5[i]->getYawError() / M_PI * 180 << std::endl;
          }
      }

      if(tag_36h11_6->isFound())
        {
          yaw_errors.push_back(tag_36h11_6->getYawError());
        }
      if(yaw_errors.size() > 0)
      {
        std::sort (yaw_errors.begin(), yaw_errors.end());
        average_yaw_error = yaw_errors[found_tag_count / 2] / M_PI * 180;
      }

      if(!found_16h5)
      {
        landing_threshold = 0.3;
      }
      else
      {
        landing_threshold = 0.1;
      }
      //check whether the landing condition is met
      if(fabs(average_landing_center_position(0)) < landing_threshold && fabs(average_landing_center_position(1)) < landing_threshold)
      {
        // std::cout << "local x: " << local_x << " target x: " << average_landing_center_position(0) << " local y: " << local_y << " target y: " << average_landing_center_position(1) << std::endl;
        if(landing_enabled)
        {
          landing_condition_met_msg.data = true;
          landing_condition_met_pub.publish(landing_condition_met_msg);

          relanding_condition_met_msg.data = false;
          relanding_condition_met_pub.publish(landing_condition_met_msg);
          
          relanding = false;

          std::cout << "error x: " << average_landing_center_position(0) << " error y: " << average_landing_center_position(1) << " landing enabled" << std::endl;
        }
      }
      else if((fabs(average_landing_center_position(0)) > reland_horizontal_threshold || fabs(average_landing_center_position(1)) > reland_horizontal_threshold) && fabs(average_landing_center_position(2)) < reland_height_min_threshold)//not close to the center at the horizontal plane but close enough at height
      {
        landing_condition_met_msg.data = false;
        landing_condition_met_pub.publish(landing_condition_met_msg);

        relanding_condition_met_msg.data = true;
        relanding_condition_met_pub.publish(landing_condition_met_msg);

        relanding = true;

        std::cout << "error x: " << average_landing_center_position(0) << " error y: " << average_landing_center_position(1) << " height: " << average_landing_center_position(2) << " landing enabled" << std::endl;
      
      }
      else
      {
        landing_condition_met_msg.data = false;
        landing_condition_met_pub.publish(landing_condition_met_msg);

        relanding_condition_met_msg.data = false;
        relanding_condition_met_pub.publish(landing_condition_met_msg);
        std::cout << "error x: " << average_landing_center_position(0) << " error y: " << average_landing_center_position(1) << std::endl;
      }

      yaw_state_msg.data = yaw_state;
      yaw_state_pub.publish(yaw_state_msg);

      setpoint_yaw = yaw_state + average_yaw_error - 90;
      setpoint_yaw_msg.data = setpoint_yaw;
      setpoint_yaw_pub.publish(setpoint_yaw_msg);

      setpoint_x = average_landing_center_position(0) + local_x;
      setpoint_y = average_landing_center_position(1) + local_y;
      // std::cout << "setpoing y: " << setpoint_y << " error y: " << average_landing_center_position(1) << " local y: " << local_y << std::endl;

      setpoint_x_msg.data = setpoint_x;
      setpoint_y_msg.data = setpoint_y;

      setpoint_x_pub.publish(setpoint_x_msg);
      setpoint_y_pub.publish(setpoint_y_msg);
    }
    else
    {
      setpoint_x_msg.data = setpoint_x;
      setpoint_y_msg.data = setpoint_y;
      setpoint_yaw_msg.data = setpoint_yaw;
      yaw_state_msg.data = 0;

      if(local_z < absolute_reland_height_min_threshold || local_z < reland_height_max_threshold)
      {
        landing_condition_met_msg.data = false;
        landing_condition_met_pub.publish(landing_condition_met_msg);

        relanding_condition_met_msg.data = true;
        relanding_condition_met_pub.publish(landing_condition_met_msg);
        std::cout << "absolute reland" << std::endl;
      }


      setpoint_x_pub.publish(setpoint_x_msg);
      setpoint_y_pub.publish(setpoint_y_msg);
      setpoint_yaw_pub.publish(setpoint_yaw_msg);
      yaw_state_pub.publish(yaw_state_msg);
    }

    

    loop_rate.sleep();
  }
}
