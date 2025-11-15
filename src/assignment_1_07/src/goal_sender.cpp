#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"


// ========= GoalSender Node =============
class GoalSender : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoalSender() : Node("goal_sender"), goal_in_progress_(false) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose_raw", 10,
      std::bind(&GoalSender::goal_callback, this, std::placeholders::_1));
  }

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (goal_in_progress_) {
      RCLCPP_WARN(get_logger(), "Goal already in progress, ignoring new one");
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = *msg;

    RCLCPP_INFO(get_logger(), "Sending goal: (%.2f, %.2f) in frame %s",
                msg->pose.position.x, msg->pose.position.y,
                msg->header.frame_id.c_str());

    goal_in_progress_ = true;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result) {
      goal_in_progress_ = false;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "✅ Navigation succeeded!"); // Add some emoji to distinguish success, failure, and cancellation in the logs
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(get_logger(), "⚠️ Navigation aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(get_logger(), "❌ Navigation canceled"); 
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Unknown result code");
          break;
      }
    };

    action_client_->async_send_goal(goal_msg, options);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  bool goal_in_progress_;

}; // End of GoalSender Node

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSender>());
  rclcpp::shutdown();
  return 0;
}
