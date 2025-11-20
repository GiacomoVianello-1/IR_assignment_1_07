#include <memory>
#include <chrono>
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

    // Subscription to corridor_active: trigger cancellation when true
    corridor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/corridor_active", 10,
      std::bind(&CancelNav2Goal::corridorCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "✅ CancelNav2Goal ready, waiting for /corridor_active...");
  }

private:
  void corridorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      RCLCPP_INFO(get_logger(), "Corridor active = true -> sending cancel request to Nav2");

      if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Action server navigate_to_pose not available");
        return;
      }

      auto future = client_->async_cancel_all_goals();
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

      if (future.valid()) {
        RCLCPP_INFO(get_logger(), "Cancel request sent successfully.");
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to send cancel request.");
      }
    } else {
      RCLCPP_INFO(get_logger(), "Corridor active = false → no cancellation");
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr corridor_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CancelNav2Goal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
