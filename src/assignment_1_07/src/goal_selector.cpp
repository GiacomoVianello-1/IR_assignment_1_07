#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "assignment_1_07/srv/get_goal.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/bool.hpp"

class GoalSelector : public rclcpp::Node {
public:
  GoalSelector()
  : Node("goal_selector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    nav2_ready_(false),
    nav2_enable_(true)   // ✅ inizializzato a true per default
  {
    // Subscription to AprilTag detections for logging purposes
    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10,
      std::bind(&GoalSelector::detectionsCallback, this, std::placeholders::_1));

    // Service to provide navigation goals
    goal_service_ = this->create_service<assignment_1_07::srv::GetGoal>(
      "/get_goal",
      std::bind(&GoalSelector::handleGetGoal, this,
                std::placeholders::_1, std::placeholders::_2));

    // Subscription to Nav2 readiness signal
    nav2_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/nav2_ready", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        nav2_ready_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Nav2 ready = %s", nav2_ready_ ? "true" : "false");
      });

    // Subscription to Nav2 enable signal (Corridor_Controller)
    nav2_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/nav2_enable", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        nav2_enable_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Nav2 enable = %s", nav2_enable_ ? "true" : "false");
      });

    target_frame_     = this->declare_parameter<std::string>("target_frame", "map");
    tag_frame_prefix_ = this->declare_parameter<std::string>("tag_frame_prefix", "tag36h11:");
    tf_timeout_sec_   = this->declare_parameter<double>("tf_timeout_sec", 0.3);

    RCLCPP_INFO(get_logger(), "✅ GoalSelector ready (target_frame=%s)", target_frame_.c_str());
  }

private:
  void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Detected %zu tags.", msg->detections.size());
  }

  void handleGetGoal(
    const std::shared_ptr<assignment_1_07::srv::GetGoal::Request> request,
    std::shared_ptr<assignment_1_07::srv::GetGoal::Response> response)
  {
    if (!nav2_ready_ || !nav2_enable_) {
      response->success = false;
      response->message = "Nav2 not ready or disabled";
      RCLCPP_WARN(get_logger(), "Service call rejected: Nav2 not ready or disabled.");
      return;
    }

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

      RCLCPP_INFO(get_logger(),
        "✅ Goal computed between tag %d and %d: (%.2f, %.2f)",
        request->tag_id_1, request->tag_id_2,
        response->goal.pose.position.x, response->goal.pose.position.y);

    } catch (const tf2::TransformException &ex) {
      response->success = false;
      response->message = std::string("TF lookup failed: ") + ex.what();
      RCLCPP_WARN(get_logger(), "⚠️ TF error computing goal: %s", ex.what());
    }
  }

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Service<assignment_1_07::srv::GetGoal>::SharedPtr goal_service_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav2_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav2_enable_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string target_frame_;
  std::string tag_frame_prefix_;
  double tf_timeout_sec_;

  bool nav2_ready_;
  bool nav2_enable_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSelector>());
  rclcpp::shutdown();
  return 0;
}
