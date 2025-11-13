#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


// ==== GoalSelector Node ====
class GoalSelector : public rclcpp::Node{
public: GoalSelector(): Node("goal_selector"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_){
    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10,
      std::bind(&GoalSelector::detectionsCallback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");
    tag_frame_prefix_ = this->declare_parameter<std::string>("tag_frame_prefix", "tag36h11:");
    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.3);
  }

private:
  void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg){
    if (msg->detections.size() < 2) {
      RCLCPP_WARN(get_logger(), "Need at least 2 detections to compute goal.");
      return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> tag_poses;
    tag_poses.reserve(msg->detections.size());

    for (const auto &det : msg->detections) {
      const std::string tag_frame = tag_frame_prefix_ + std::to_string(det.id);

      try {
        // Use TimePointZero for the latest available transform and a timeout
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
        RCLCPP_WARN(get_logger(), "TF lookup failed %s -> %s: %s", tag_frame.c_str(), target_frame_.c_str(), ex.what());
      }
    }

    if (tag_poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "Not enough tag poses after TF lookup.");
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = target_frame_;
    goal.pose.position.x = (tag_poses[0].pose.position.x + tag_poses[1].pose.position.x) / 2.0;
    goal.pose.position.y = (tag_poses[0].pose.position.y + tag_poses[1].pose.position.y) / 2.0;
    goal.pose.position.z = 0.0;

    // Neutral orientation: facing forward in the target frame (may be adjusted as needed)
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    goal_pub_->publish(goal);
    RCLCPP_INFO(get_logger(), "Published goal at (%.3f, %.3f) in %s",
                goal.pose.position.x, goal.pose.position.y, target_frame_.c_str());
  }

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string target_frame_;
  std::string tag_frame_prefix_;
  double tf_timeout_sec_;

}; // GoalSelector Class

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSelector>());
  rclcpp::shutdown();
  return 0;
}
