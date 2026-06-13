#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/byte.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "ar_interfaces/action/move_twist.hpp"
#include "robotlibpc/cpp/controller/pid.hpp"
#include "qos_profiles.hpp"
#include <mutex>

using MoveTwist = ar_interfaces::action::MoveTwist;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveTwist>;

class OdomPIDServer : public rclcpp::Node
{
public:
    OdomPIDServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{});

    rclcpp_action::Server<MoveTwist>::SharedPtr action_server_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr prox_front_sub_;

    float odom_dist_x_start{0.0f};
    float odom_dist_y_start{0.0f};

    std::atomic<bool>  odom_fresh_{false};
    std::atomic<bool>  prox_front_triggered_{false};
    std::atomic<float> odom_dist_x_{0.0f};
    std::atomic<float> odom_dist_y_{0.0f};

    std::atomic<bool> is_action_finished{false};
    std::atomic<bool> is_movex_done{false};
    std::atomic<bool> is_movey_done{false};

    double x_kp;
    double x_ki;
    double x_kd;
    double y_kp;
    double y_ki;
    double y_kd;
    PID x_pid;
    PID y_pid;
    double max_vel_;
    double sample_time_;
    double dist_tol_;
    void publishVel(float vx, float vy);
    void odomCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void proxFrontCallback(const std_msgs::msg::Byte::SharedPtr msg);

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveTwist::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleMove> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandleMove> goal_handle);
    void execute(const std::shared_ptr<GoalHandleMove> goal_handle);
};