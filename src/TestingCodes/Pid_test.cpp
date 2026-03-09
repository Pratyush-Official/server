#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/UInt16MultiArray.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ar_interfaces/action/movement.hpp"

class GoToSpearNode : public rclcpp::Node
{
public:
    using Movement = ar_interfaces::action::Movement;
    using GoalHandleMovement = rclcpp_action::ClientGoalHandle<Movement>;

    GoToSpearNode()
    : Node("go_to_spear_node")
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

        // Subscribers
        sick_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/sick_data", qos_profile,
            std::bind(&GoToSpearNode::sickCallback, this, std::placeholders::_1)
        );

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_data", qos_profile,
            std::bind(&GoToSpearNode::odomCallback, this, std::placeholders::_1)
        );

        // Action client
        client_ptr_spear_ = rclcpp_action::create_client<Movement>(
            this, "/movement_pid"
        );

        RCLCPP_INFO(this->get_logger(), "GoToSpear node initialized");

        // Optional: periodically send PID goals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GoToSpearNode::sendGoal, this)
        );
    }

private:
    rclcpp_action::Client<Movement>::SharedPtr client_ptr_spear_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sick_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<uint16_t> sick_data_;
    nav_msgs::msg::Odometry odom_msg_;
    bool goal_sent_ = false;

    void sickCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        sick_data_ = msg->data;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_msg_ = *msg;
    }

    void sendGoal()
    {
        if (!client_ptr_spear_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "PID action server not available yet");
            return;
        }

        if (goal_sent_) return;

        auto goal_msg = Movement::Goal();
        // Example: populate goal from SICK data
        goal_msg.setpoint_sick = {0.0, 0.0, 0.0, 0.0};
        goal_msg.tolerance_sick = 0.05;
        goal_msg.timeout_sick = 5.0;

        auto goal_options = rclcpp_action::Client<Movement>::SendGoalOptions();
        goal_options.goal_response_callback = 
            [](std::shared_ptr<GoalHandleMovement> gh) {
                if (!gh) RCLCPP_ERROR(rclcpp::get_logger("go_to_spear_node"), "Goal rejected");
                else RCLCPP_INFO(rclcpp::get_logger("go_to_spear_node"), "Goal accepted");
            };
        goal_options.feedback_callback = 
            [](GoalHandleMovement::SharedPtr, const std::shared_ptr<const Movement::Feedback> fb) {
                RCLCPP_INFO(rclcpp::get_logger("go_to_spear_node"), "PID feedback: %f", fb->current_position);
            };
        goal_options.result_callback = 
            [this](const GoalHandleMovement::WrappedResult &result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                else
                    RCLCPP_WARN(this->get_logger(), "Goal failed");
                goal_sent_ = false; // ready to send next goal
            };

        client_ptr_spear_->async_send_goal(goal_msg, goal_options);
        goal_sent_ = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToSpearNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
