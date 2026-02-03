#include "slam_toolbox/slam_toolbox_multirobot.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
MultiRobotSlamToolbox::MultiRobotSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options),
  tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) // Initialize TF broadcaster
/*****************************************************************************/
{
    // Subscribe to raw laser scan topic
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 50, std::bind(&MultiRobotSlamToolbox::laserCallback, this, std::placeholders::_1));
}

/*****************************************************************************/
void MultiRobotSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
    // Process raw laser scans
    Pose2 pose;
    if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
        RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
        return;
    }

    LaserRangeFinder * laser = getLaser(scan);
    if (!laser) {
        RCLCPP_WARN(get_logger(), "Failed to create laser device for %s; discarding scan", scan->header.frame_id.c_str());
        return;
    }

    // Add scan to SLAM system
    addScan(laser, scan, pose);

    // Publish the robot pose in the map frame
    publishPoseToTF(pose);
}

/*****************************************************************************/
void MultiRobotSlamToolbox::publishPoseToTF(const karto::Pose2& pose)
/*****************************************************************************/
{
    geometry_msgs::msg::TransformStamped transform_msg;

    // Broadcast the dynamic transform (map -> base_link)
    transform_msg.header.stamp = this->now();
    transform_msg.header.frame_id = "map";
    transform_msg.child_frame_id = "base_link";
    transform_msg.transform.translation.x = pose.GetX();
    transform_msg.transform.translation.y = pose.GetY();
    transform_msg.transform.translation.z = 0.0;
    double theta = pose.GetHeading();
    transform_msg.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta / 2), cos(theta / 2)));

    // Send the transform
    tf_broadcaster_->sendTransform(transform_msg);

    // For static transforms, use the same TransformBroadcaster but only publish once.
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "odom";
    static_transform.child_frame_id = "base_footprint";
    static_transform.transform.translation.x = 0.0;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;  // No rotation

    tf_broadcaster_->sendTransform(static_transform);  // Publish static transform
}

/*****************************************************************************/
LaserRangeFinder * MultiRobotSlamToolbox::getLaser(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
/*****************************************************************************/
{
    const std::string & frame = scan->header.frame_id;
    if (lasers_.find(frame) == lasers_.end()) {
        try {
            lasers_[frame] = laser_assistant_->toLaserMetadata(*scan);
            dataset_->Add(lasers_[frame].getLaser(), true);
        } catch (tf2::TransformException & e) {
            RCLCPP_ERROR(get_logger(), "Failed to compute laser pose[%s], aborting initialization (%s)", frame.c_str(), e.what());
            return nullptr;
        }
    }

    return lasers_[frame].getLaser();
}

/*****************************************************************************/
bool MultiRobotSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
    if (req->match_type == procType::LOCALIZE_AT_POSE) {
        RCLCPP_WARN(get_logger(), "Requested a localization deserialization in non-localization mode.");
        return false;
    }

    return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

}  // namespace slam_toolbox
