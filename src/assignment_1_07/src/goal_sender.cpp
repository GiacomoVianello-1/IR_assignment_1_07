#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "assignment_1_07/srv/get_goal.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class GoalSender : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoalSender() : Node("goal_sender"), goal_in_progress_(false), nav2_ready_(false), nav2_enable_(true) {
    // Action client for Nav2
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Client for the GetGoal service
    goal_client_ = this->create_client<assignment_1_07::srv::GetGoal>("/get_goal");

    // Subscription al segnale dell’orchestrator
    nav2_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/nav2_ready", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        nav2_ready_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Nav2 ready = %s", nav2_ready_ ? "true" : "false");
      });

    // Subscription al segnale di abilitazione (Corridor_Controller)
    nav2_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/nav2_enable", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        nav2_enable_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Nav2 enable = %s", nav2_enable_ ? "true" : "false");
      });

    // Declare parameters for tag IDs
    tag_id_1_ = this->declare_parameter<int>("tag_id_1", 1);
    tag_id_2_ = this->declare_parameter<int>("tag_id_2", 10);

    // Timer to periodically request goals
    timer_ = this->create_wall_timer(2s, std::bind(&GoalSender::requestGoal, this));

    RCLCPP_INFO(get_logger(), "✅ GoalSender ready (tag1=%d, tag2=%d)", tag_id_1_, tag_id_2_);
  }

private:
  void requestGoal() {
    // Wait for Nav2 readiness and enable
    if (!nav2_ready_ || !nav2_enable_) {
      RCLCPP_DEBUG(get_logger(), "⏳ Waiting for Nav2 orchestrator/enable signal...");
      return;
    }

    // Don't request a new goal if one is in progress
    if (goal_in_progress_) {
      RCLCPP_DEBUG(get_logger(), "Navigation in progress, skipping goal request.");
      return;
    }

    if (!goal_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "⚠️ Service /get_goal not available yet.");
      return;
    }

    auto request = std::make_shared<assignment_1_07::srv::GetGoal::Request>();
    request->tag_id_1 = tag_id_1_;
    request->tag_id_2 = tag_id_2_;

    goal_client_->async_send_request(
      request,
      [this](rclcpp::Client<assignment_1_07::srv::GetGoal>::SharedFuture future) {
        auto result = future.get();
        if (!result->success) {
          RCLCPP_WARN(get_logger(), "⚠️ Goal request failed: %s", result->message.c_str());
          return;
        }

        // Avoid sending the same goal repeatedly
        double dx = result->goal.pose.position.x - last_goal_.pose.position.x;
        double dy = result->goal.pose.position.y - last_goal_.pose.position.y;
        if (std::hypot(dx, dy) < 0.1) {
          RCLCPP_DEBUG(get_logger(), "Goal unchanged, skipping.");
          return;
        }

        last_goal_ = result->goal;
        sendGoal(result->goal);
      }
    );
  }

  void sendGoal(const geometry_msgs::msg::PoseStamped &goal_msg){
    if (!nav2_ready_ || !nav2_enable_) {
      RCLCPP_INFO(get_logger(), "⏳ Nav2 not ready or disabled, skipping goal send.");
      return;
    }

    if (goal_in_progress_) {
      RCLCPP_DEBUG(get_logger(), "⚠️ Goal already in progress");
      return;
    }

    if (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_ERROR(get_logger(), "❌ NavigateToPose action server not available");
      return;
    }

    RCLCPP_INFO(get_logger(),
      "📨 Sending goal: (%.2f, %.2f) in frame %s",
      goal_msg.pose.position.x,
      goal_msg.pose.position.y,
      goal_msg.header.frame_id.c_str()
    );

    goal_in_progress_ = true;

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal_msg;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.result_callback =
      [this](const GoalHandleNavigateToPose::WrappedResult &result) {
        goal_in_progress_ = false;

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "🎯 Navigation succeeded! Shut down nodes");
            rclcpp::shutdown(); // Optional: shut down after reaching goal
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "⚠️ Navigation aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "❌ Navigation canceled");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "❓ Unknown result code");
            break;
        }
      };

    action_client_->async_send_goal(nav_goal, options);
  }

  // Parameters 
  int tag_id_1_;
  int tag_id_2_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Client<assignment_1_07::srv::GetGoal>::SharedPtr goal_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav2_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav2_enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_in_progress_;
  bool nav2_ready_;
  bool nav2_enable_;
  geometry_msgs::msg::PoseStamped last_goal_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSender>());
  rclcpp::shutdown();
  return 0;
}
