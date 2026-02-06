#include "servers/yaw_server.hpp"

PIDServerNode::PIDServerNode()
    : Node("pid_server_node")
    {

        // this->declare_parameter<double>("pid.vx.kp", 0.07);
        // this->declare_parameter<double>("pid.vx.ki", 0.0);
        // this->declare_parameter<double>("pid.vx.kd", 0.0);

        // this->declare_parameter<double>("pid.vy.kp", 0.1);
        // this->declare_parameter<double>("pid.vy.ki", 0.1);
        // this->declare_parameter<double>("pid.vy.kd", 0.0);

        this->declare_parameter<double>("pid.w.kp",  0.01);
        this->declare_parameter<double>("pid.w.ki",  0.1);
        this->declare_parameter<double>("pid.w.kd",  0.0);

        // pid_vx_ = std::make_unique<PID>(
            // this->get_parameter("pid.vx.kp").as_double(),
            // this->get_parameter("pid.vx.ki").as_double(),
            // this->get_parameter("pid.vx.kd").as_double(),
            // -1.0, 1.0f, 10, PROPORTIONAL_ON_ERROR, DIRECT, AUTOMATIC);
        // 
 
        // pid_vy_ = std::make_unique<PID>(
            // this->get_parameter("pid.vy.kp").as_double(),
            // this->get_parameter("pid.vy.ki").as_double(),
            // this->get_parameter("pid.vy.kd").as_double(),
            // -1.0f, 1.0f, 10, PROPORTIONAL_ON_ERROR, DIRECT, AUTOMATIC);

        pid_w_ = std::make_unique<PID>(
            this->get_parameter("pid.w.kp").as_double(),
            this->get_parameter("pid.w.ki").as_double(),
            this->get_parameter("pid.w.kd").as_double(),
            -1.0f, 1.0f, 10, PROPORTIONAL_ON_ERROR, DIRECT, AUTOMATIC);

        yaw_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/yaw_data", 10,
            std::bind(&PIDServerNode::yawCallback, this, std::placeholders::_1)
        );
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        action_server_ = rclcpp_action::create_server<Movement>(
            this,
            "/movement_pid",
            std::bind(&PIDServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PIDServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&PIDServerNode::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "PID mecanum server started");
    }

    // ---------------- CALLBACKS ----------------
void PIDServerNode::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4) {
        sick_vals_ = msg->data;
        sick_ready_ = true;
    }
}

void PIDServerNode::pidLoop()
{
    RCLCPP_DEBUG(this->get_logger(), "PID Loop entered");
    //if (!pid_vx_) return;
    if (!sick_ready_ || !goal_active_) return;

    float FL = sick_vals_[0];
    //float FR = sick_vals_[1];
    float L  = sick_vals_[2];
    float R  = sick_vals_[3];
    float W =  yaw;

    float sp_FL = goal_sp_[0];
    //float sp_FR = goal_sp_[1];
    float sp_L  = goal_sp_[2];
    float sp_R  = goal_sp_[3];
    float sp_w = goal_sp_w;

    //float sp_yaw = (sick_vals_[0] + sick_vals_[1]) * 0.5f;
    //error_vx = ((FL-sp_FL) + (FR - sp_FR)) * 0.5f;
    error_vx = (FL-sp_FL);
    error_vy = (L - sp_L) - (R - sp_R);
    error_w = 

    // pid_vx_->input = error_vx; 
    // pid_vy_->input = error_vy;
    pid_w_->input = error_w;

    // pid_vx_->setpoint = 0.0f;
    // pid_vy_->setpoint = 0.0f;
    pid_w_->setpoint = 0.0f;

    pid_vx_->compute();
    pid_vy_->compute();

    geometry_msgs::msg::Twist cmd;

    cmd.linear.x  = -(pid_vx_->output);
    cmd.linear.y  = (pid_vy_->output);
    //cmd.angular.z = (pid_w_->output);

    RCLCPP_INFO(this->get_logger(),
                 "PID Outputs x: ");
                 
    cmd_pub_->publish(cmd);
}

    // ---------------- ACTION SERVER ----------------
rclcpp_action::GoalResponse PIDServerNode::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Movement::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received PID goal request");
    if (goal->setpoint_sick.size() >= 4) {
        true_goal_sp_.resize(4);  // ensure size
        for (size_t i = 0; i < 4; ++i) {
            true_goal_sp_[i] = static_cast<float>(goal->setpoint_sick[i])/10.0f; // convert mm -> m if needed
        }
        goal_active_ = true;
        RCLCPP_INFO(this->get_logger(), "PID goal received and setpoints updated to : %.2f, %.2f, %.2f, %.2f", true_goal_sp_[0], true_goal_sp_[1], true_goal_sp_[2], true_goal_sp_[3]);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RCLCPP_WARN(this->get_logger(), "PID goal rejected: setpoint_sick has less than 4 elements");
        return rclcpp_action::GoalResponse::REJECT;
    }
}

// void PIDServerNode::trajectory_tracking(){

//     float step = 5.0f;
//     float error_to_goal = 0.0f;
//     if (goal_active_ && sick_ready_) {
//         if(fabs(true_goal_sp_[0] - sick_vals_[0]) < 0.05f &&
//            fabs(true_goal_sp_[1] - sick_vals_[1]) < 0.05f &&
//            fabs(true_goal_sp_[2] - sick_vals_[2]) < 0.05f &&
//            fabs(true_goal_sp_[3] - sick_vals_[3]) < 0.05f)
//         {
//             for(int i = 0; i < 4; i++){
//                 goal_sp_[i] = true_goal_sp_[i];
//                 printf("Final setpoint given\n");
//             }
//         }
//         else{
//             for(int i = 0; i < 4; i++){
//                 error_to_goal = true_goal_sp_[i] - sick_vals_[i];
//                 goal_sp_[i] += std::copysign(std::fmin(step, fabs(error_to_goal)), error_to_goal) ; // move closer each loop
//                 printf("Intermediate setpoint given\n");
//             }
//         }
//     }
// }

rclcpp_action::CancelResponse PIDServerNode::handle_cancel(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    goal_active_ = false;
    RCLCPP_WARN(this->get_logger(), "PID goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PIDServerNode::handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting PID goal execution");
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void PIDServerNode::execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    auto result = std::make_shared<Movement::Result>();
    geometry_msgs::msg::Twist cmd;
    rclcpp::Rate rate(100); // 10ms loop
    //const float tol = 0.5f;
    while (rclcpp::ok() && goal_active_ && sick_ready_) {
        pidLoop();
        printf("PID Loop executed\n");
        float error_vx = ((sick_vals_[0] - true_goal_sp_[0]) + (sick_vals_[1] - true_goal_sp_[1])) * 0.5f;
        float error_vy = (sick_vals_[2] - true_goal_sp_[2]) - (sick_vals_[3] - true_goal_sp_[3]);

        if (std::fabs(error_vx) < 0.5f &&
            std::fabs(error_vy) < 0.5f &&
            std::fabs(error_w)  < 0.5f) {
            printf("PID goal reached within tolerance\n");

            pid_
            cmd.linear.x  = -0.0f;
            cmd.linear.y  = 0.0f;
            cmd_pub_->publish(cmd);
            break;
        }
        rate.sleep();
        
    }
    goal_active_ = false;

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "PID goal reached");
}

// ---------------- MAIN ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDServerNode>());
    rclcpp::shutdown();
    return 0;
}
