#include "slam_toolbox/slam_toolbox_multirobot.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
MultiRobotSlamToolbox::MultiRobotSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options)
/*****************************************************************************/
{
    // Subscribes to raw laser scan topic
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&MultiRobotSlamToolbox::laserCallback, this, std::placeholders::_1));
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

    addScan(laser, scan, pose);
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
