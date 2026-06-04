#include "servers/sick_pid_server.hpp"
#include "qos_profiles.hpp"

SickPIDServer::SickPIDServer()
    : Node("pid_server_node")
    {

        this->declare_parameter<double>("pid.vx.kp", 0.07);
        this->declare_parameter<double>("pid.vx.ki", 0.0);
        this->declare_parameter<double>("pid.vx.kd", 0.0);

        this->declare_parameter<double>("pid.vy.kp", 0.1);
        this->declare_parameter<double>("pid.vy.ki", 0.0);
        this->declare_parameter<double>("pid.vy.kd", 0.0);
    
        this->declare_parameter("dist_tol",100.0);

        pid_vx_ = std::make_unique<PID>(
            this->get_parameter("pid.vx.kp").as_double(),
            this->get_parameter("pid.vx.ki").as_double(),
            this->get_parameter("pid.vx.kd").as_double(),
            -0.2f, 0.2f, 10, 2.0f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);
        

        pid_vy_ = std::make_unique<PID>(
            this->get_parameter("pid.vy.kp").as_double(),
            this->get_parameter("pid.vy.ki").as_double(),
            this->get_parameter("pid.vy.kd").as_double(),
            -0.2f, 0.2f, 10, 2.0f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

        sick_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sick_data", sensor_qos(),
            std::bind(&SickPIDServer::sickCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());

        action_server_ = rclcpp_action::create_server<Movement>(
            this,
            "/movement_sick",
            std::bind(&SickPIDServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SickPIDServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&SickPIDServer::handle_accepted, this, std::placeholders::_1)
        );

        pid_vx_->setTunings(0.01f, 0.0f, 0.0f);
        pid_vy_->setTunings(0.009000f, 0.0f, 0.0f);
        RCLCPP_INFO(this->get_logger(), "Sick PID Server started");

        pid_vx_->init(); //This resets a timer
        pid_vy_->init(); //This resets a timer

        service_complete = false;
        x_reached = false;
        y_reached = false;
    }

void SickPIDServer::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data.size() >= 4) {
        sick_vals_ = msg->data;
        sick_ready_ = true;
    }
}

rclcpp_action::GoalResponse SickPIDServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Movement::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received PID goal request");

    if (goal_active_) {
        RCLCPP_WARN(this->get_logger(), "PID busy, rejecting new goal");

        resetVel();
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->setpoint_sick.size() >= 4) {
        true_goal_sp_.resize(4);  // ensure size
        for (size_t i = 0; i < 4; ++i) {
            true_goal_sp_[i] = static_cast<float>(goal->setpoint_sick[i])/10.0f;

            x_reached = false;
            y_reached = false;

        }

    pid_vx_->setpoint = true_goal_sp_[0];
    pid_vy_->setpoint = true_goal_sp_[2];

    service_complete = false;
    goal_active_ = true;
    
    resetVel();    

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    } 
    
    else {
        RCLCPP_WARN(this->get_logger(), "PID goal rejected: setpoint_sick has less than 4 elements");

        resetVel();
        return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse SickPIDServer::handle_cancel(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    goal_active_ = false;
    RCLCPP_WARN(this->get_logger(), "PID goal canceled");

    resetVel();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SickPIDServer::handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting PID goal execution"); 
    resetVel();
    pid_vx_->reset();
    pid_vy_->reset();
    // FIX 2: call init() after reset() to stamp lastTime_ to now,
    // otherwise the first compute() sees a huge timeChange and fires instantly
    pid_vx_->init();
    pid_vy_->init();

    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void SickPIDServer::execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[SickPIDServer] Executing");

    auto result = std::make_shared<Movement::Result>();

    rclcpp::Rate rate(100); 
    while (rclcpp::ok() && goal_active_ && !sick_ready_) {
        rate.sleep();
    }

    if (!rclcpp::ok() || !goal_active_) {
        result->success = false;
        goal_handle->succeed(result);
        return;
    }

    while (rclcpp::ok() && goal_active_ && !service_complete)
    {
        float vx = 0.0f;
        float vy = 0.0f;

        float FL = 0.0f;
        float L = 0.0f;
        float sp_FL = 0.0f;
        float sp_L = 0.0f;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!sick_ready_ || true_goal_sp_.size() < 4) {
                rate.sleep();
                continue;
            }
            FL = sick_vals_[0];
            L = sick_vals_[2];
            sp_FL = true_goal_sp_[0];
            sp_L  = true_goal_sp_[2];
        }

        pid_vx_->input = FL;
        pid_vy_->input = L;

        pid_vx_->setpoint = sp_FL;
        pid_vy_->setpoint = sp_L;

        pid_vx_->compute();
        pid_vy_->compute();

        vx = -(pid_vx_->output);
        vy = (pid_vy_->output);

        RCLCPP_INFO(this->get_logger(),
            "[SickPIDServer] Goal received : X_SETPOINT=%.1f X_INPUT=%.1f Y_SETPOINT=%.1f Y_INPUT=%.1f",
            pid_vx_->setpoint, pid_vx_->input, pid_vy_->setpoint, pid_vy_->input);

        if (fabs(pid_vx_->setpoint - pid_vx_->input) < 2.0f)
        {
            vx = 0.0f;
            x_reached = true;
            printf("x true\n");
        }

        if (fabs(pid_vy_->setpoint - pid_vy_->input) < 2.0f)
        {
            vy = 0.0f;
            y_reached = true;
            printf("y true\n");
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z = 0.0f;
        cmd_pub_->publish(cmd);

        if (x_reached && y_reached)
        {
            service_complete = true;

            goal_active_ = false;
            resetVel();
            pid_vx_->reset();
            pid_vy_->reset();
            RCLCPP_INFO(this->get_logger(), "[SickPIDServer] Completed");
            result->success = true;
        }

        rate.sleep();
    }

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "PID goal reached");
}

void SickPIDServer::resetVel()
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    cmd_pub_->publish(cmd);
}

// ---------------- MAIN ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SickPIDServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}