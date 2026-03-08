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

        pid_vx_ = std::make_unique<PID>(
            this->get_parameter("pid.vx.kp").as_double(),
            this->get_parameter("pid.vx.ki").as_double(),
            this->get_parameter("pid.vx.kd").as_double(),
            -0.2f, 0.2f, 10, 0.01f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);
        

        pid_vy_ = std::make_unique<PID>(
            this->get_parameter("pid.vy.kp").as_double(),
            this->get_parameter("pid.vy.ki").as_double(),
            this->get_parameter("pid.vy.kd").as_double(),
            -0.2f, 0.2f, 10, 0.01f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

        sick_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sick_data", sensor_qos(),
            std::bind(&SickPIDServer::sickCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());

        graph_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sick_error", sensor_qos());

        // slider_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //     "/pid_gains", 10,
        //     std::bind(&SickPIDServer::sliderCallback, this, std::placeholders::_1));

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
    }

SickPIDServer::~SickPIDServer() {
    RCLCPP_INFO(this->get_logger(), "Shutting down SickPIDServer");
    goal_active_ = false; // stop any running PID loops
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd_pub_->publish(cmd);
    
    pid_vx_->reset();
    pid_vy_->reset();
}

    // ---------------- CALLBACKS ----------------
void SickPIDServer::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data.size() >= 4) {
        sick_vals_ = msg->data;
        sick_ready_ = true;
        
    }
}

void SickPIDServer::sliderCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4) {
        slider_data_ = msg->data;
        printf("Slider data obtained: ");
        printf("%.3f ", slider_data_[0]);
        printf("\n");
    }
}

void SickPIDServer::pidLoop()
{
    geometry_msgs::msg::Twist cmd;
    std_msgs::msg::Float32MultiArray graph_out;
    RCLCPP_DEBUG(this->get_logger(), "PID Loop entered");
    // if (!pid_vx_) return;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!sick_ready_ || !goal_active_ || true_goal_sp_.size() < 4) return;
        
        FL = sick_vals_[0];
        L = sick_vals_[1];
        sp_FL = true_goal_sp_[0];
        sp_L  = true_goal_sp_[1];
    }

    error_vx = (sp_FL - FL);
    error_vy = (sp_L - L);

    pid_vx_->input = FL;
    pid_vy_->input = L;

    pid_vx_->setpoint = sp_FL;
    pid_vy_->setpoint = sp_L;

    pid_vx_->compute();
    pid_vy_->compute();
    
    cmd.linear.x  = -(pid_vx_->output);
    cmd.linear.y  = (pid_vy_->output);

    RCLCPP_INFO(this->get_logger(), "PID input x: %.2f", FL);

    graph_out.data.clear();
    graph_out.data.push_back(pid_vx_->input);
    graph_out.data.push_back(pid_vx_->setpoint);
    graph_out.data.push_back(pid_vx_->output);
    // graph_out.data.push_back(pid_vy_->input);
    // graph_out.data.push_back(pid_vy_->setpoint);
                 
    graph_pub_->publish(graph_out);
    cmd_pub_->publish(cmd);
}

    // ---------------- ACTION SERVER ----------------
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
            true_goal_sp_[i] = static_cast<float>(goal->setpoint_sick[i])/10.0f; // convert mm -> m if needed
        }
        goal_active_ = true;
        // RCLCPP_INFO(this->get_logger(), "PID goal received and setpoints updated to : %.2f, %.2f, %.2f, %.2f", true_goal_sp_[0], true_goal_sp_[1], true_goal_sp_[2], true_goal_sp_[3]);
        resetVel();    
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RCLCPP_WARN(this->get_logger(), "PID goal rejected: setpoint_sick has less than 4 elements");

        resetVel();
        return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse SickPIDServer::handle_cancel(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    goal_active_ = false;
    RCLCPP_WARN(this->get_logger(), "PID goal canceled");

    resetVel(); // Stop the robot immediately on cancel
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SickPIDServer::handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting PID goal execution"); 
    resetVel(); // Ensure robot is stopped at start of new goal
    pid_vx_->reset();
    pid_vy_->reset();

    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void SickPIDServer::execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    int settle_count = 0;
    auto result = std::make_shared<Movement::Result>();
    geometry_msgs::msg::Twist cmd;
    rclcpp::Rate rate(25); // 10ms loop
    //const float tol = 0.5f;

    float error_vx;
    float error_vy;
    while (rclcpp::ok() && goal_active_ && sick_ready_) {
        // Compute errors
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            error_vx = true_goal_sp_[0] - sick_vals_[0];
            error_vy = true_goal_sp_[1] - sick_vals_[1];
        }

        if (std::fabs(error_vx) < 0.2f && std::fabs(error_vy) < 0.2f) {
            resetVel();
            settle_count++;
            pid_vx_->reset();
            pid_vy_->reset();

            if(settle_count >= 5) {
                goal_active_ = false;
                resetVel();
                pid_vx_->reset();
                pid_vy_->reset();
                break;
            }
        }
        else{
            settle_count = 0;
            pidLoop(); //Only run pid loop when not settled
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
