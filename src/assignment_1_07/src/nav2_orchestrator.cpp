#include <chrono>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class Nav2Orchestrator : public rclcpp::Node {
public:
  Nav2Orchestrator() : Node("nav2_orchestrator") {
    init_x_ = this->declare_parameter<double>("initial_x", 0.0);
    init_y_ = this->declare_parameter<double>("initial_y", 0.0);
    init_yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);

    cli_localization_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
      "/lifecycle_manager_localization/manage_nodes");
    cli_navigation_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
      "/lifecycle_manager_navigation/manage_nodes");

    initpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    ready_pub_    = this->create_publisher<std_msgs::msg::Bool>("/nav2_ready", 1);
  }

  void run() {
    // 1. Startup localization
    bool loc_ok = send_startup(cli_localization_, "localization");

    // 2. Publish initial pose
    publish_initial_pose();
    std::this_thread::sleep_for(2s);

    // 3. Startup navigation
    bool nav_ok = send_startup(cli_navigation_, "navigation");

    // 4. If everything is ok, publish readiness on /nav2_ready topic (useful for other nodes)
    std_msgs::msg::Bool msg;
    msg.data = loc_ok && nav_ok;
    ready_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Nav2 readiness published: %s", msg.data ? "true" : "false");
  }

private:
  bool send_startup(const rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr &cli,
                    const std::string &name) {
    if (!cli->wait_for_service(10s)) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", name.c_str());
      return false;
    }
    auto req = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    req->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
    auto fut = cli->async_send_request(req);
    if (rclcpp::spin_until_future_complete(shared_from_this(), fut) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto res = fut.get();
      RCLCPP_INFO(get_logger(), "STARTUP on %s -> success=%s",
                  name.c_str(), res->success ? "true" : "false");
      return res->success;
    }
    return false;
  }

  void publish_initial_pose() {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    msg.pose.pose.position.x = init_x_;
    msg.pose.pose.position.y = init_y_;
    msg.pose.pose.orientation.w = std::cos(init_yaw_ / 2.0);
    msg.pose.pose.orientation.z = std::sin(init_yaw_ / 2.0);
    initpose_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Initial pose published: x=%.2f y=%.2f yaw=%.2f rad", init_x_, init_y_, init_yaw_);
  }

  double init_x_, init_y_, init_yaw_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr cli_localization_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr cli_navigation_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Orchestrator>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
