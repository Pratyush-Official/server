#ifndef ODOM_PID_SERVER_HPP
#define ODOM_PID_SERVER_HPP

#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "ar_interfaces/action/movement.hpp"
#include "robotlibpc/cpp/controller/pid.hpp"

class OdomPIDServer : public rclcpp::Node
{
public:

    OdomPIDServer();
    using Movement = ar_interfaces::action::Movement;
    using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp_action::Server<Movement>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr slider_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr graph_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr setpoint_pub;
    std_msgs::msg::Float32MultiArray setpoint_;
    rclcpp::TimerBase::SharedPtr pid_timer_;

    std::unique_ptr<PID> pid_vx_;
    std::unique_ptr<PID> pid_vy_;
    std::unique_ptr<PID> pid_w_;

    float error_vx{0.0f};
    float error_vy{0.0f};
    float error_w {0.0f};

    double vx_kp, vx_ki, vx_kd;
    double vy_kp, vy_ki, vy_kd;
    double w_kp,  w_ki,  w_kd;

    std::vector<float> odom_vals_{0.0f, 0.0f};
    std::vector<int32_t> data{0, 0};
    bool odom_ready_{false};

    float odom_start1_{0.0f};
    float odom_start2_{0.0f};
    float odom_vx{0.0f};
    float odom_vy{0.0f};
    float sp_vx  {0.0f};
    float sp_vy  {0.0f};

    std::mutex data_mutex_;

    std::vector<float> true_goal_sp_{0.0f, 0.0f};
    bool goal_active_{false};

    std::vector<float> slider_data_{0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t RESET_YAW_BYTE{0x42};
    float error_parallel_yaw{0.0f};

    void odomCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void sliderCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void pidLoop();
    void resetVel();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Movement::Goal> goal
    );

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMovement> goal_handle
    );

    void handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle);
    void execute(const std::shared_ptr<GoalHandleMovement> goal_handle);
};

#endif