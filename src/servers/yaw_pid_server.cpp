#include "servers/yaw_pid_server.hpp"
#include "qos_profiles.hpp"

YawPIDServer::YawPIDServer()
    : Node("pid_server_node")
    {
        this->declare_parameter<double>("pid.w.kp", 4.0);
        this->declare_parameter<double>("pid.w.ki", 5.0);
        this->declare_parameter<double>("pid.w.kd", 1.0);

        pid_w_ = std::make_unique<PID>(
            this->get_parameter("pid.w.kp").as_double(),
            this->get_parameter("pid.w.ki").as_double(),
            this->get_parameter("pid.w.kd").as_double(),
            -0.7f, 0.7f, 10, 0.005f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
        // graph_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sick_error", sensor_qos());
        bno_sub = this->create_subscription<std_msgs::msg::Float32>("/imu_data", sensor_qos(), std::bind(&YawPIDServer::imuCallback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<Movement>(
            this,
            "/yaw_pid",
            std::bind(&YawPIDServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&YawPIDServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&YawPIDServer::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Yaw PID Server started");

        pid_w_->init(); //This resets a timer
    }

    // ---------------- CALLBACKS ----------------
void YawPIDServer::imuCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    imu_ready_ = true;
    imu_data = msg->data;
    // RCLCPP_INFO(this->get_logger(),"Current imu position %f", imu_data);
}

float YawPIDServer::toYawPidSetpoint(float setpoint, float current, AngularDirectionType dir)
{
    float angleDiff = wrapAngle(setpoint) - wrapAngle(current);
    switch (dir) {
        case AUTO:
            if (angleDiff > M_2_PI)       angleDiff -= M_2_PI;
            else if (angleDiff < -M_PI) angleDiff += M_2_PI;
            break;
        case ANTICLOCKWISE:
            if (angleDiff < 0) angleDiff += M_2_PI;
            break;
        case CLOCKWISE:
            if (angleDiff > 0) angleDiff -= M_2_PI;
            break;
    }
    return current + angleDiff;
}

void YawPIDServer::pidLoop()
{
    geometry_msgs::msg::Twist cmd;
    std_msgs::msg::Float32MultiArray graph_out;
    RCLCPP_DEBUG(this->get_logger(), "PID Loop entered");

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!imu_ready_ || !goal_active_) return;
        
        yaw = imu_data;
    }

    error_yaw = (sp_yaw - yaw);

    pid_w_->input = yaw;
    pid_w_->setpoint = sp_yaw;
    pid_w_->compute();
    cmd.angular.z  = (pid_w_->output);

    // RCLCPP_INFO(this->get_logger(), "PID input x: %.2f", FL);

    cmd_pub_->publish(cmd);
}

    // ---------------- ACTION SERVER ----------------
rclcpp_action::GoalResponse YawPIDServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Movement::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received PID goal request");

    if (goal_active_) {
        RCLCPP_WARN(this->get_logger(), "PID busy, rejecting new goal");

        resetVel();
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    {    
        std::lock_guard<std::mutex> lock(data_mutex_);
        float data_ob = goal->setpoint_yaw;
        
        sp_yaw = toYawPidSetpoint(data_ob, imu_data); // CMADT_AUTO by default
        goal_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Setpoint: %f", sp_yaw);
        
    }
        // RCLCPP_INFO(this->get_logger(), "PID goal received and setpoints updated to : %.2f, %.2f, %.2f, %.2f", true_goal_sp_[0], true_goal_sp_[1], true_goal_sp_[2], true_goal_sp_[3]);
        resetVel();    
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    // else {
    //     RCLCPP_WARN(this->get_logger(), "PID goal rejected: setpoint_imu has less than 4 elements");

    //     resetVel();
    //     return rclcpp_action::GoalResponse::REJECT;
    // }
}

rclcpp_action::CancelResponse YawPIDServer::handle_cancel(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    goal_active_ = false;
    RCLCPP_WARN(this->get_logger(), "PID goal canceled");

    resetVel(); // Stop the robot immediately on cancel
    return rclcpp_action::CancelResponse::ACCEPT;
}

void YawPIDServer::handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting PID goal execution"); 
    resetVel(); // Ensure robot is stopped at start of new goal

    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void YawPIDServer::execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    auto result = std::make_shared<Movement::Result>();
    geometry_msgs::msg::Twist cmd;
    rclcpp::Rate rate(100); // 10ms loop

    while (rclcpp::ok() && goal_active_ && imu_ready_) {
        
        // RCLCPP_INFO(this->get_logger(), "Loop is running with error_vx: %.3f, error_vy: %.3f", error_vx, error_vy);

        if (pid_w_->is_allowable_error)  {
                goal_active_ = false;

                RCLCPP_INFO(this->get_logger(), "PID goal reached");

                resetVel();
                pid_w_->reset();
                break;
            }

        else{
            pidLoop();
        }
        rate.sleep();
    }
        
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "PID goal reached");
}
void YawPIDServer::resetVel()
{
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0f;
    cmd_pub_->publish(cmd);
}

// ---------------- MAIN ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YawPIDServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
