#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "assignment_1_07/srv/get_goal.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class GoalSelector : public rclcpp::Node {
public:
  GoalSelector(): Node("goal_selector"),tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),goal_computed_(false){
    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10,
      std::bind(&GoalSelector::detectionsCallback, this, std::placeholders::_1));

    goal_service_ = this->create_service<assignment_1_07::srv::GetGoal>(
      "/get_goal",
      std::bind(&GoalSelector::handleGetGoal, this, std::placeholders::_1, std::placeholders::_2));

    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    tag_frame_prefix_ = this->declare_parameter<std::string>("tag_frame_prefix", "tag36h11:");
    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.3);

    RCLCPP_INFO(get_logger(), "!!-- GoalSelector ready --!!");
  }

private:
  void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg){
    if (goal_computed_) return;

    if (msg->detections.size() < 2) {
        RCLCPP_WARN(get_logger(), "Need at least 2 detections.");
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> tag_poses;
    tag_poses.reserve(2);

    for (int i = 0; i < 2; i++) {
      const auto &det = msg->detections[i];
      const std::string tag_frame = tag_frame_prefix_ + std::to_string(det.id);

      try {
        auto tf_tag_to_target = tf_buffer_.lookupTransform(target_frame_, tag_frame, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_));
        geometry_msgs::msg::PoseStamped pose;
        pose.header = tf_tag_to_target.header;
        pose.pose.position.x = tf_tag_to_target.transform.translation.x;
        pose.pose.position.y = tf_tag_to_target.transform.translation.y;
        pose.pose.orientation = tf_tag_to_target.transform.rotation;
        tag_poses.push_back(pose);

      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "TF error %s -> %s: %s", tag_frame.c_str(), target_frame_.c_str(), ex.what());
        return;
      }
    }

    current_goal_.header.stamp = now();
    current_goal_.header.frame_id = target_frame_;
    current_goal_.pose.position.x = (tag_poses[0].pose.position.x + tag_poses[1].pose.position.x) * 0.5;
    current_goal_.pose.position.y = (tag_poses[0].pose.position.y + tag_poses[1].pose.position.y) * 0.5;
    current_goal_.pose.orientation.w = 1.0;

    goal_computed_ = true;

    RCLCPP_INFO(get_logger(), "Goal computed.");
  }

  void handleGetGoal(
    const std::shared_ptr<assignment_1_07::srv::GetGoal::Request> request,
    std::shared_ptr<assignment_1_07::srv::GetGoal::Response> response)
  {
    const std::string tag_frame1 = tag_frame_prefix_ + std::to_string(request->tag_id_1);
    const std::string tag_frame2 = tag_frame_prefix_ + std::to_string(request->tag_id_2);

    try {
      auto tf1 = tf_buffer_.lookupTransform(target_frame_, tag_frame1, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_));
      auto tf2 = tf_buffer_.lookupTransform(target_frame_, tag_frame2, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_));

      response->goal.header.stamp = now();
      response->goal.header.frame_id = target_frame_;
      response->goal.pose.position.x = (tf1.transform.translation.x + tf2.transform.translation.x) * 0.5;
      response->goal.pose.position.y = (tf1.transform.translation.y + tf2.transform.translation.y) * 0.5;
      response->goal.pose.orientation.w = 1.0;

      response->success = true;
      response->message = "Goal computed successfully";

      RCLCPP_INFO(get_logger(), "✅ Goal computed between tag %d and %d", request->tag_id_1, request->tag_id_2);

    } catch (const tf2::TransformException &ex) {
      response->success = false;
      response->message = std::string("TF lookup failed: ") + ex.what();
      RCLCPP_WARN(get_logger(), "⚠️ TF error computing goal: %s", ex.what());
    }
  }



  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Service<assignment_1_07::srv::GetGoal>::SharedPtr goal_service_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string target_frame_;
  std::string tag_frame_prefix_;
  double tf_timeout_sec_;

  bool goal_computed_;
  geometry_msgs::msg::PoseStamped current_goal_;

}; // class GoalSelector

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSelector>());
  rclcpp::shutdown();
  return 0;
}
