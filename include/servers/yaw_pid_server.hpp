#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include "ar_interfaces/action/rotate_exec.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include "robotlibpc/cpp/controller/pid.hpp"

namespace YawPIDServer
{

using RotateExec = ar_interfaces::action::RotateExec;
using GoalHandleRotate = rclcpp_action::ServerGoalHandle<RotateExec>;

class YawPIDServer : public rclcpp::Node
{
public:
    explicit YawPIDServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  
    rclcpp_action::Server<RotateExec>::SharedPtr action_server_;

   
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_sub_;

  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_yaw_pub_;

    PID yaw_pid_;

    
    double yaw_kp_{0.0};
    double yaw_ki_{0.0};
    double yaw_kd_{0.0};
    double yaw_tolerance_rad_{0.01};
    double max_omega_p_{1.0};
    double max_omega_n_{-1.0};
    int    sample_time_{20};          

    
    float  imu_yaw_{0.0f};
    bool   imu_received_{false};

   
    float normalizeAngle(float angle);

 
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const RotateExec::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleRotate> goal_handle);

    void handleAccepted(
        const std::shared_ptr<GoalHandleRotate> goal_handle);

    
    void execute(const std::shared_ptr<GoalHandleRotate> goal_handle);

    
    void imuCallback(const std_msgs::msg::Float32::SharedPtr msg);
};

} 