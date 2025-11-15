#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

// ========== GoalSelector Node =============
class GoalSelector : public rclcpp::Node {
public:
  GoalSelector(): Node("goal_selector"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), goal_computed_(false){

    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10,
      std::bind(&GoalSelector::detectionsCallback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_raw", 10);

    // Timer to publish goal at 5 seconds interval
    publish_timer_ = this->create_wall_timer(
      5s,
      std::bind(&GoalSelector::publishGoal, this));

    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    tag_frame_prefix_ = this->declare_parameter<std::string>("tag_frame_prefix", "tag36h11:");
    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.3);
  }

private:
  void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    if (msg->detections.size() < 2) {
      RCLCPP_WARN(get_logger(), "Need at least 2 detections to compute goal.");
      return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> tag_poses;
    tag_poses.reserve(msg->detections.size());

    for (const auto &det : msg->detections) {
      const std::string tag_frame = tag_frame_prefix_ + std::to_string(det.id);

      try {
        const auto tf_tag_to_target = tf_buffer_.lookupTransform(
          target_frame_, tag_frame, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_));

        geometry_msgs::msg::PoseStamped pose_target;
        pose_target.header.stamp = tf_tag_to_target.header.stamp;
        pose_target.header.frame_id = target_frame_;
        pose_target.pose.position.x = tf_tag_to_target.transform.translation.x;
        pose_target.pose.position.y = tf_tag_to_target.transform.translation.y;
        pose_target.pose.position.z = tf_tag_to_target.transform.translation.z;
        pose_target.pose.orientation = tf_tag_to_target.transform.rotation;

        tag_poses.push_back(pose_target);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "TF lookup failed %s -> %s: %s",
                    tag_frame.c_str(), target_frame_.c_str(), ex.what());
      }
    }

    if (tag_poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "Not enough tag poses after TF lookup.");
      return;
    }

    // Compute goal as midpoint between first two tag poses
    current_goal_.header.stamp = now();
    current_goal_.header.frame_id = target_frame_;
    current_goal_.pose.position.x = (tag_poses[0].pose.position.x + tag_poses[1].pose.position.x) / 2.0;
    current_goal_.pose.position.y = (tag_poses[0].pose.position.y + tag_poses[1].pose.position.y) / 2.0;
    current_goal_.pose.position.z = 0.0;
    current_goal_.pose.orientation.w = 1.0;

    goal_computed_ = true;
    // RCLCPP_INFO(get_logger(), "Goal computed at (%.3f, %.3f)", current_goal_.pose.position.x, current_goal_.pose.position.y);
  }

  void publishGoal() {
    if (goal_computed_) {
      goal_pub_->publish(current_goal_);
      RCLCPP_INFO(get_logger(), "Republished goal at (%.3f, %.3f) in %s",
                  current_goal_.pose.position.x, current_goal_.pose.position.y,
                  target_frame_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "No goal computed yet, waiting...");
    }
  }

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string target_frame_;
  std::string tag_frame_prefix_;
  double tf_timeout_sec_;

  bool goal_computed_;
  geometry_msgs::msg::PoseStamped current_goal_;
  
}; // End of GoalSelector Node

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSelector>());
  rclcpp::shutdown();
  return 0;
}
