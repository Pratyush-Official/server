#include "servers/align.hpp"

AlignServerNode::AlignServerNode()
    : Node("align_server_node")
    {

        this->declare_parameter<double>("pid.w.kp",  0.01);
        this->declare_parameter<double>("pid.w.ki",  0.1);
        this->declare_parameter<double>("pid.w.kd",  0.0);

        pid_w_ = std::make_unique<PID>(
            this->get_parameter("pid.w.kp").as_double(),
            this->get_parameter("pid.w.ki").as_double(),
            this->get_parameter("pid.w.kd").as_double(),
            -1.0f, 1.0f, 10, PROPORTIONAL_ON_ERROR, DIRECT, AUTOMATIC);

        sick_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sick_data", 10,
            std::bind(&AlignServerNode::sickCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        action_server_ = rclcpp_action::create_server<Movement>(
            this,
            "/movement_pid",
            std::bind(&AlignServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AlignServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&AlignServerNode::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "PID mecanum server started");
}

    // ---------------- CALLBACKS ----------------
void AlignServerNode::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4) {
        sick_vals_ = msg->data;
        sick_ready_ = true;
    }
}

void AlignServerNode::pidLoop()
{
    RCLCPP_DEBUG(this->get_logger(), "PID Loop entered");
    if (!sick_ready_ || !goal_active_) return;
    float FL = sick_vals_[0];
    float FR = sick_vals_[1];
    float sp_FL = goal_sp_[0];
    float sp_FR = goal_sp_[1];
    float sp_yaw = 0.0f;
    //float sp_yaw = (sick_vals_[0] + sick_vals_[1]) * 0.5f;
    error_w = FL - FR;
    
    pid_w_->input  = error_w;
    pid_w_->setpoint  = sp_yaw;
    pid_w_->compute();
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = (pid_w_->output);
    RCLCPP_INFO(this->get_logger(),
                 "PID Outputs x: ");
                 
    cmd_pub_->publish(cmd);
}

// ---------------- ACTION SERVER ----------------
rclcpp_action::GoalResponse AlignServerNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Movement::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received PID goal request");
    if (goal->setpoint_sick.size() >= 4) {
        goal_sp_.resize(4);  // ensure size
        for (size_t i = 0; i < 4; ++i) {
            goal_sp_[i] = static_cast<float>(goal->setpoint_sick[i])/10.0f; // convert mm -> m if needed
        }
        goal_active_ = true;
        RCLCPP_INFO(this->get_logger(), "PID goal received and setpoints updated to : %.2f, %.2f", goal_sp_[0], goal_sp_[1]);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RCLCPP_WARN(this->get_logger(), "PID goal rejected: setpoint_sick has less than 4 elements");
        return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse AlignServerNode::handle_cancel(const std::shared_ptr<GoalHandleMovement>)
{
    goal_active_ = false;
    RCLCPP_WARN(this->get_logger(), "PID goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AlignServerNode::handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting PID goal execution");
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void AlignServerNode::execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    rclcpp::Rate rate(100); // 10ms loop
    //const float tol = 0.5f;
    while (rclcpp::ok() && goal_active_ && sick_ready_) {
        pidLoop();
        printf("PID Loop executed\n");
        // float error_vx = ((sick_vals_[0] - goal_sp_[0]) + (sick_vals_[1] - goal_sp_[1])) * 0.5f;
        // float error_vy = (sick_vals_[2] - goal_sp_[2]) - (sick_vals_[3] - goal_sp_[3]);
        // float error_w  = (sick_vals_[0] - goal_sp_[0]) - (sick_vals_[1] - goal_sp_[1]);
        // if (std::fabs(error_vx) < tol &&
        //     std::fabs(error_vy) < tol &&
        //     std::fabs(error_w)  < tol) {
        //     printf("PID goal reached within tolerance\n");
        //     break;
        // }
        rate.sleep();
        
    }
    goal_active_ = false;
    //goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "PID goal reached");
}

// ---------------- MAIN ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlignServerNode>());
    rclcpp::shutdown();
    return 0;
}
