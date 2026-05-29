#include "servers/yaw_pid_server.hpp"
#include "qos_profiles.hpp"
#include <cmath>
#include <thread>
#include <chrono>

namespace YawPIDServer
{

YawPIDServer::YawPIDServer(const rclcpp::NodeOptions & options)
: Node("rotate_exec_server", options)
, yaw_pid_()          
{
    
    this->declare_parameter("yaw_kp",            0.8);
    this->declare_parameter("yaw_ki",            0.0);
    this->declare_parameter("yaw_kd",            0.05);
    this->declare_parameter("yaw_tolerance_rad", 0.01);
    this->declare_parameter("max_omega_p",        0.7);
    this->declare_parameter("max_omega_n",       -0.7);
    this->declare_parameter("sample_time_ms",     20);

    yaw_kp_           = this->get_parameter("yaw_kp").as_double();
    yaw_ki_           = this->get_parameter("yaw_ki").as_double();
    yaw_kd_           = this->get_parameter("yaw_kd").as_double();
    yaw_tolerance_rad_= this->get_parameter("yaw_tolerance_rad").as_double();
    max_omega_p_      = this->get_parameter("max_omega_p").as_double();
    max_omega_n_      = this->get_parameter("max_omega_n").as_double();
    sample_time_      = this->get_parameter("sample_time_ms").as_int();

    
    yaw_pid_.setTunings(
        static_cast<float>(yaw_kp_),
        static_cast<float>(yaw_ki_),
        static_cast<float>(yaw_kd_));
    yaw_pid_.setOutputLimits(
        static_cast<float>(max_omega_n_),
        static_cast<float>(max_omega_p_));
    yaw_pid_.setSampleTime(static_cast<uint16_t>(sample_time_));
    yaw_pid_.setMode(AUTOMATIC);
    yaw_pid_.init();

    
    imu_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/imu_data_received", sensor_qos(),
        std::bind(&YawPIDServer::imuCallback, this, std::placeholders::_1));

    
    cmd_yaw_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rotate_server/cmd_vel", cmd_vel_qos());

    action_server_ = rclcpp_action::create_server<RotateExec>(
        this,
        "rotate_exec",
        std::bind(&YawPIDServer::handleGoal,     this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&YawPIDServer::handleCancel,   this, std::placeholders::_1),
        std::bind(&YawPIDServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "[YawPIDServer] Ready  kp=%.3f ki=%.3f kd=%.3f tol=%.4f rad",
                yaw_kp_, yaw_ki_, yaw_kd_, yaw_tolerance_rad_);
}

void YawPIDServer::imuCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    imu_yaw_      = normalizeAngle(msg->data);
    imu_received_ = true;
}

// Normalize angle to [-180, 180]

float YawPIDServer::normalizeAngle(float angle)
{
    while (angle >  M_PI) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -M_PI) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

rclcpp_action::GoalResponse YawPIDServer::handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const RotateExec::Goal> goal)
{
    RCLCPP_INFO(get_logger(),
                "[YawPIDServer] Goal received as target_angle=%.4f rad",
                goal->target_angle);

    if (!imu_received_) {
        RCLCPP_ERROR(get_logger(),
                     "[YawPIDServer] No IMU data yet REJECT");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse YawPIDServer::handleCancel(
    const std::shared_ptr<GoalHandleRotate> /*goal_handle*/)
{
    RCLCPP_WARN(get_logger(), "[YawPIDServer] Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void YawPIDServer::handleAccepted(
    const std::shared_ptr<GoalHandleRotate> goal_handle)
{
    std::thread{
        std::bind(&YawPIDServer::execute, this, std::placeholders::_1),
        goal_handle
    }.detach();
}

void YawPIDServer::execute(
    const std::shared_ptr<GoalHandleRotate> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    float setpoint = normalizeAngle(imu_yaw_ + goal->target_angle);

    RCLCPP_INFO(get_logger(),
                "[YawPIDServer] Executing  imu_yaw=%.4f  setpoint=%.4f rad",
                imu_yaw_, setpoint);

    yaw_pid_.setpoint = setpoint;
    yaw_pid_.input    = imu_yaw_;
    yaw_pid_.init();

    auto feedback = std::make_shared<RotateExec::Feedback>();
    auto result   = std::make_shared<RotateExec::Result>();

    rclcpp::Rate rate(1000 / sample_time_);  // restore the rate sleep

    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling())
        {
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.angular.z = 0.0f;
            cmd_yaw_pub_->publish(stop_cmd);

            result->success   = false;
            result->final_yaw = imu_yaw_;
            goal_handle->canceled(result);
            RCLCPP_WARN(get_logger(), "[YawPIDServer] Goal cancelled");
            return;
        }

        yaw_pid_.input = imu_yaw_;
        yaw_pid_.compute();

        float error = normalizeAngle(setpoint - imu_yaw_);

        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = yaw_pid_.output;
        cmd_yaw_pub_->publish(cmd);

        feedback->current_yaw_error = error;
        feedback->current_yaw       = imu_yaw_;
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(get_logger(),
                    "[YawPIDServer] error=%.4f  omega=%.4f",
                    error, yaw_pid_.output);

        if (std::fabs(error) <= static_cast<float>(yaw_tolerance_rad_))
        {
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.angular.z = 0.0f;
            cmd_yaw_pub_->publish(stop_cmd);

            result->success   = true;
            result->final_yaw = imu_yaw_;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(),
                        "[YawPIDServer] Succeeded  final_yaw=%.4f rad",
                        imu_yaw_);
            return;
        }

        rate.sleep();
    }

    result->success   = false;
    result->final_yaw = imu_yaw_;
    goal_handle->abort(result);
}

} 

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YawPIDServer::YawPIDServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}