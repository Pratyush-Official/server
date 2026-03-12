#include "servers/odom_pid_server.hpp"
#include "qos_profiles.hpp"

OdomPIDServer::OdomPIDServer()
    : Node("pid_server_node")
    {

        this->declare_parameter<double>("odom.vx.kp", 0.6);
        this->declare_parameter<double>("odom.vx.ki", 0.1);
        this->declare_parameter<double>("odom.vx.kd", 0.0);

        this->declare_parameter<double>("odom.vy.kp", 0.1);
        this->declare_parameter<double>("odom.vy.ki", 0.1);
        this->declare_parameter<double>("odom.vy.kd", 0.0);

        pid_vx_ = std::make_unique<PID>(
            this->get_parameter("odom.vx.kp").as_double(),
            this->get_parameter("odom.vx.ki").as_double(),
            this->get_parameter("odom.vx.kd").as_double(),
            -0.7f, 0.7f, 10, 0.1f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

        pid_vy_ = std::make_unique<PID>(
            this->get_parameter("odom.vy.kp").as_double(),
            this->get_parameter("odom.vy.ki").as_double(),
            this->get_parameter("odom.vy.kd").as_double(),
            -0.7f, 0.7f, 10, 0.1f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());

        odom_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/odom_data", sensor_qos(),
            std::bind(&OdomPIDServer::odomCallback, this, std::placeholders::_1)
        );

        action_server_ = rclcpp_action::create_server<Movement>(
            this,
            "/movement_odom",
            std::bind(&OdomPIDServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&OdomPIDServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&OdomPIDServer::handle_accepted, this, std::placeholders::_1)
        );

        pid_vx_->init();
        pid_vy_->init();

        RCLCPP_INFO(this->get_logger(), "Odom PID server started");
    }

// ---------------- CALLBACKS ----------------
void OdomPIDServer::odomCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data.size() >= 2) {
        data = msg->data;

        const float wheel_diameter = 0.0574f;
        const int counts_per_rev = 4000;

        float dist1 = (data[1] * 3.1416f * wheel_diameter) / counts_per_rev; //pins were swapped, will change this later
        float dist2 = (data[0] * 3.1416f * wheel_diameter) / counts_per_rev;

        odom_vals_[0] = dist1;
        odom_vals_[1] = dist2;

        odom_ready_ = true;

        printf("Odom 1: %f Odom 2: %f \n", dist1, dist2);
    }
}

// ---------------- PID LOOP ----------------
void OdomPIDServer::pidLoop()
{
    geometry_msgs::msg::Twist cmd;
    RCLCPP_DEBUG(this->get_logger(), "PID Loop entered");

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!odom_ready_ || !goal_active_ || true_goal_sp_.size() < 2) return;

        odom_vx = odom_vals_[0] - odom_start1_;
        odom_vy = odom_vals_[1] - odom_start2_;
        sp_vx   = true_goal_sp_[0];
        sp_vy   = true_goal_sp_[1];
    }

    error_vx = sp_vx - odom_vx;

    pid_vx_->input    = odom_vx;
    pid_vx_->setpoint = sp_vx;
    pid_vx_->compute();

    // RCLCPP_INFO(this->get_logger(),
    //             "PID Loop: input=%.4f  setpoint=%.4f  error=%.4f  output=%.4f",
    //             odom_vx, sp_vx, error_vx, pid_vx_->output);

    cmd.linear.x = -(pid_vx_->output);
    cmd_pub_->publish(cmd);
}

// ---------------- ACTION SERVER ----------------
rclcpp_action::GoalResponse OdomPIDServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Movement::Goal> goal)
{
    (void) uuid;
    RCLCPP_INFO(this->get_logger(), "Received PID goal request");

    if (goal_active_) {
        RCLCPP_WARN(this->get_logger(), "PID busy, rejecting new goal");
        resetVel();
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->setpoint_odom.size() >= 2) {
        true_goal_sp_.resize(2);
        for (size_t i = 0; i < 2; ++i) {
            true_goal_sp_[i] = static_cast<float>(goal->setpoint_odom[i]);
        }

        goal_active_ = true;

        RCLCPP_INFO(this->get_logger(), "PID goal received: setpoints = %.4f, %.4f",
                    true_goal_sp_[0], true_goal_sp_[1]);
        resetVel();
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RCLCPP_WARN(this->get_logger(), "PID goal rejected: setpoint_odom has less than 2 elements");
        resetVel();
        return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse OdomPIDServer::handle_cancel(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    (void) goal_handle;
    goal_active_ = false;
    RCLCPP_WARN(this->get_logger(), "PID goal canceled");
    resetVel();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OdomPIDServer::handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting PID goal execution");
    resetVel();
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void OdomPIDServer::execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    auto result = std::make_shared<Movement::Result>();
    rclcpp::Rate rate(100); // 10ms loop

    // Wait until odom data arrives
    while (rclcpp::ok() && !odom_ready_) {
        rate.sleep();
    }

    // Snapshot odom at goal start
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        odom_start1_ = odom_vals_[0];
        odom_start2_ = odom_vals_[1];
    }

    pid_vx_->reset();
    pid_vy_->reset();

    while (rclcpp::ok() && goal_active_ && odom_ready_) {

        if (pid_vx_->is_allowable_error) {
            goal_active_ = false;

            RCLCPP_INFO(this->get_logger(), "PID goal reached");

            resetVel();
            pid_vx_->reset();
            pid_vy_->reset();
            break;
        } else {
            pidLoop();
        }

        rate.sleep();
    }

    goal_active_ = false;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "PID goal succeeded");
}

void OdomPIDServer::resetVel()
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
    auto node = std::make_shared<OdomPIDServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}