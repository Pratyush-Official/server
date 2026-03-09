#ifndef YAW_PID_SERVER_HPP
#define YAW_PID_SERVER_HPP

#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "qos_profiles.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "ar_interfaces/action/movement.hpp"
#include "robotlibpc/cpp/controller/pid.hpp"

class YawPIDServer : public rclcpp::Node
{
public:

    YawPIDServer();
    using Movement = ar_interfaces::action::Movement;
    using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;

private:

    const float TWO_PI = 2.0f * M_PI;
    const float PI = M_PI;
    // ---------------- ROS interfaces ----------------
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp_action::Server<Movement>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bno_sub;
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr slider_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr graph_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr setpoint_pub;
    std_msgs::msg::Float32MultiArray setpoint_;
    rclcpp::TimerBase::SharedPtr pid_timer_;

    // ---------------- PID controllers ----------------
    std::unique_ptr<PID> pid_w_;
    std::mutex data_mutex_;

    float sp_yaw;
    float yaw;
    float imu_data;
    float error_yaw  = 0.0f;
    double w_kp,  w_ki,  w_kd;

    // SICK DATA 
    // std::vector<float> sick_vals_{0.0f, 0.0f, 0.0f, 0.0f}; // FL, FR, L, R
    bool imu_ready_{false};

    // ---------------- CLIENT SETPOINTS ----------------
    float true_goal_sp_ = 0.0f; // FL, FR, L, R
    bool goal_active_{false};

    // ---------------- Callbacks ----------------
    void imuCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void pidLoop();
    void resetVel();

    inline float wrapAngle(float angle)
    {
        // Wrap to [0, 2*pi)
        angle = fmod(angle + PI, TWO_PI);
        if (angle < 0.0f)
            angle += TWO_PI;
    
        // Shift to [-pi, pi)
        return angle - PI;
    }

    // ---------------- Action server handlers ----------------
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

#endif // PID_SERVER_NODE_HPP
