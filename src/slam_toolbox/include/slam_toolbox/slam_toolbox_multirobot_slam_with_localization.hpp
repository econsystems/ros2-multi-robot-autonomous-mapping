/*
 * multirobot_slam_toolbox
 * Copyright Work Modifications (c) 2023, Achala Athukorala
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#ifndef SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_
#define SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_

#include <memory>
#include "slam_toolbox/slam_toolbox_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace slam_toolbox
{

/**
 * @brief MultiRobotSlamToolbox class extends SlamToolbox for multi-robot SLAM.
 *
 * This class provides functionality for handling laser scans, localizing multiple robots,
 * and publishing poses as TF transforms.
 */
class MultiRobotSlamToolbox : public SlamToolbox
{
public:
  /**
   * @brief Construct a new MultiRobotSlamToolbox object.
   * 
   * @param options Node options for configuration.
   */
  explicit MultiRobotSlamToolbox(rclcpp::NodeOptions options);
  
  /**
   * @brief Destructor for cleaning up resources.
   */
  ~MultiRobotSlamToolbox() {};

protected:
  /**
   * @brief Gets the LaserRangeFinder object corresponding to the given laser scan.
   * 
   * @param scan The laser scan message.
   * @return LaserRangeFinder* The corresponding LaserRangeFinder object.
   */
  LaserRangeFinder * getLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);

  /**
   * @brief Callback for processing laser scan data.
   * 
   * This method is overridden from SlamToolbox to handle incoming laser scan messages.
   * 
   * @param scan The laser scan message.
   */
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;

  /**
   * @brief Callback for deserializing the pose graph.
   * 
   * @param request_header The request header.
   * @param req The deserialization request.
   * @param resp The deserialization response.
   * @return true if deserialization is successful, false otherwise.
   */
  bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp) override;

  /**
   * @brief Publishes the robot's pose as a transform to the TF system.
   * 
   * This method publishes the robot's pose in the map frame, using a TransformBroadcaster.
   * 
   * @param pose The robot's pose in 2D.
   */
  void publishPoseToTF(const karto::Pose2& pose);

  // Subscriber to laser scan messages
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

  // TF broadcaster to send transforms
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace slam_toolbox

#endif   // SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_
