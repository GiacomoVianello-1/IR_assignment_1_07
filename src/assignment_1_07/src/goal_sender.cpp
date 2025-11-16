#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "assignment_1_07/srv/get_goal.hpp"

using namespace std::chrono_literals;

class GoalSender : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoalSender()
        : Node("goal_sender"), goal_in_progress_(false)
    {
        // Action client per Nav2
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Client per il servizio custom
        goal_client_ = this->create_client<assignment_1_07::srv::GetGoal>("/get_goal");

        // Richiedi il goal una sola volta
        timer_ = this->create_wall_timer(
            2s, std::bind(&GoalSender::requestGoal, this)
        );
    }

private:
    void requestGoal()
    {
        if (!goal_client_->wait_for_service(5s)) {
            RCLCPP_WARN(get_logger(), "⚠️ Service /get_goal not available yet.");
            return;
        }

        auto request = std::make_shared<assignment_1_07::srv::GetGoal::Request>();

        goal_client_->async_send_request(
            request,
            [this](rclcpp::Client<assignment_1_07::srv::GetGoal>::SharedFuture future) {
                auto result = future.get();
                sendGoal(result->goal);
            }
        );

        timer_->cancel();
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped &goal_msg)
    {
        if (goal_in_progress_) {
            RCLCPP_WARN(get_logger(), "⚠️ Goal already in progress");
            return;
        }

        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(get_logger(), "❌ NavigateToPose action server not available");
            return;
        }

        RCLCPP_INFO(get_logger(),
            "📨 Sending goal: (%.2f, %.2f, %.2f) in frame %s",
            goal_msg.pose.position.x,
            goal_msg.pose.position.y,
            goal_msg.pose.position.z,
            goal_msg.header.frame_id.c_str()
        );

        goal_in_progress_ = true;

        // Costruzione corretta del messaggio di goal Nav2
        NavigateToPose::Goal nav_goal;
        nav_goal.pose = goal_msg;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult &result) {
                goal_in_progress_ = false;

                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "🎯 Navigation succeeded!");
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

        // Finalmente corretto!
        action_client_->async_send_goal(nav_goal, options);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Client<assignment_1_07::srv::GetGoal>::SharedPtr goal_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_in_progress_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalSender>());
    rclcpp::shutdown();
    return 0;
}
