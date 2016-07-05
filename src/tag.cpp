#include <tag.h>
#include <stdexcept>

Tag::Tag()
{
	position_camera_frame_ << 0, 0, 0;
	position_drone_frame_ << 0, 0, 0;
	orientation_camera_frame_ = Eigen::Quaternion<double>(1, 0, 0, 0);
	orientation_drone_frame_ = Eigen::Quaternion<double>(1, 0, 0, 0);
	to_landing_center_translation_ << 0, 0, 0;
	landing_center_position_ << 0, 0, 0;
	found_ = false;
	to_landing_center_translation_set_ = false;
}


void Tag::updateTagState(const geometry_msgs::Pose tag_pose)
{
	position_camera_frame_ = Eigen::Vector3d(tag_pose.position.x, tag_pose.position.y, tag_pose.position.z);
	orientation_camera_frame_ = Eigen::Quaternion<double>(tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z);
	found_ = true;
}


void Tag::setToLandingCenterTranslation(const Eigen::Vector3d trans)
{
	to_landing_center_translation_ = trans;
	to_landing_center_translation_set_ = true;
}

double Tag::getYawError()
{
	return atan2(2*(orientation_drone_frame_.w() * orientation_drone_frame_.z() + orientation_drone_frame_.x() * orientation_drone_frame_.y()), 1 - 2 * (orientation_drone_frame_.y() * orientation_drone_frame_.y() + orientation_drone_frame_.z() * orientation_drone_frame_.z()));
	// return orientation_drone_frame_.toRotationMatrix().eulerAngles(0, 1, 2)(2);
}

void Tag::calculateDroneFramePosition(const Eigen::Matrix3d camera_to_drone_transformation)
{
	position_drone_frame_ = camera_to_drone_transformation * position_camera_frame_;
}

void Tag::calculateDroneFrameOrientation(const Eigen::Matrix3d camera_to_drone_transformation)
{
	orientation_drone_frame_ = camera_to_drone_transformation * orientation_camera_frame_.toRotationMatrix();
}
void Tag::setMissing()
{
	found_ = false;
}

void Tag::setFound()
{
	found_ = true;
}
Eigen::Vector3d Tag::getLandingCenterPosition()
{
	if(!to_landing_center_translation_set_)
	{
		throw std::runtime_error("To landing center translation is not set!");
	}
	// orientation_drone_frame_ = camera_to_drone_transformation * orientation_camera_frame_.toRotationMatrix();
	// position_drone_frame_ = camera_to_drone_transformation * position_camera_frame_;
	landing_center_position_ = orientation_drone_frame_.matrix() * to_landing_center_translation_ + position_drone_frame_;
	return landing_center_position_;
}
Eigen::Quaternion<double> Tag::getLandingCenterOrientation()
{
	// orientation_drone_frame_ = camera_to_drone_transformation * orientation_camera_frame_.toRotationMatrix();
	return orientation_drone_frame_;
}