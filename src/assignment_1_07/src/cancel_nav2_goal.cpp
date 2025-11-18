#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class CancelNav2Goal : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  CancelNav2Goal() : Node("cancel_nav2_goal") {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Publisher for corridor_active topic: to notify GoalSender to pause navigation
    corridor_pub_ = this->create_publisher<std_msgs::msg::Bool>("/corridor_active", 10);

    // Wait a moment to ensure the action server is available
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server navigate_to_pose not available");
      return;
    }

    // Cancel all goals 
    RCLCPP_INFO(get_logger(), "Sending cancel request to Nav2...");
    auto future = client_->async_cancel_all_goals();
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

    if (future.valid()) {
      RCLCPP_INFO(get_logger(), "Cancel request sent successfully.");

      // Publish corridor_active = true
      std_msgs::msg::Bool msg;
      msg.data = true;
      corridor_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "Published corridor_active=true");

      // Timer for publishing corridor_active = false after 5 seconds
      timer_ = this->create_wall_timer(
        5s,
        [this]() {
          std_msgs::msg::Bool msg;
          msg.data = false;
          corridor_pub_->publish(msg);
          RCLCPP_INFO(get_logger(), "Published corridor_active=false");
          timer_->cancel(); // deactivate timer after one shot
        });
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to send cancel request.");
    }
    
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr corridor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CancelNav2Goal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
