#include "servers/martial_pid.hpp"

PIDServerNode::PIDServerNode()
    : Node("pid_server_node")
    {

        this->declare_parameter<double>("pid.vx.kp", 0.07);
        this->declare_parameter<double>("pid.vx.ki", 0.0);
        this->declare_parameter<double>("pid.vx.kd", 0.0);

        this->declare_parameter<double>("pid.vy.kp", 0.1);
        this->declare_parameter<double>("pid.vy.ki", 0.1);
        this->declare_parameter<double>("pid.vy.kd", 0.0);

        // this->declare_parameter<double>("pid.w.kp",  0.01);
        // this->declare_parameter<double>("pid.w.ki",  0.1);
        // this->declare_parameter<double>("pid.w.kd",  0.0);

        pid_vx_ = std::make_unique<PID>(
            this->get_parameter("pid.vx.kp").as_double(),
            this->get_parameter("pid.vx.ki").as_double(),
            this->get_parameter("pid.vx.kd").as_double(),
            -0.2f, 0.2f, 10, PROPORTIONAL_ON_ERROR, DIRECT, AUTOMATIC);
        

        pid_vy_ = std::make_unique<PID>(
            this->get_parameter("pid.vy.kp").as_double(),
            this->get_parameter("pid.vy.ki").as_double(),
            this->get_parameter("pid.vy.kd").as_double(),
            -0.2f, 0.2f, 10, PROPORTIONAL_ON_ERROR, DIRECT, AUTOMATIC);

        // pid_w_ = std::make_unique<PID>(
        //     this->get_parameter("pid.w.kp").as_double(),
        //     this->get_parameter("pid.w.ki").as_double(),
        //     this->get_parameter("pid.w.kd").as_double(),
        //     -1.0f, 1.0f, 10, PROPORTIONAL_ON_ERROR, DIRECT, AUTOMATIC);

        sick_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sick_data", 10,
            std::bind(&PIDServerNode::sickCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        graph_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pid_tune", 10);

        slider_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pid_gains", 10,
            std::bind(&PIDServerNode::sliderCallback, this, std::placeholders::_1));

        setpoint_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("setpoint_yaw", 10);

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

void PIDServerNode::sliderCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4) {
        slider_data_ = msg->data;
        printf("Slider data: ");
        printf("%.3f ", slider_data_[0]);
        printf("\n");
    }
}

void PIDServerNode::pidLoop()
{

    RCLCPP_DEBUG(this->get_logger(), "PID Loop entered");
    // if (!pid_vx_) return;
    if (!sick_ready_ || !goal_active_) return;

    pid_vx_->setTunings(0.01f, 0.0f, 0.0f);
    // pid_vx_->setTunings(slider_data_[0], slider_data_[1], slider_data_[2]);

    // pid_vy_->setTunings(slider_data_[0], slider_data_[1], slider_data_[2]);
    pid_vy_->setTunings(0.009000f, 0.0f, 0.0f);
    // printf("Slider data kp in pid loop: %.3f", slider_data_[0]);
    // trajectory_tracking();
    float FL = sick_vals_[0];
    //float FR = sick_vals_[1];
    float L  = sick_vals_[1];
    //float R  = sick_vals_[3];

    // float sp_FL = slider_data_[3];
    float sp_FL = true_goal_sp_[0];
    //float sp_FR = true_goal_sp_[1];
    // float sp_L = slider_data_[3];
    float sp_L  = true_goal_sp_[1];
    // float sp_R  = true_goal_sp_[3];

    //float sp_yaw = (sick_vals_[0] + sick_vals_[1]) * 0.5f;
    //error_vx = ((FL-sp_FL) + (FR - sp_FR)) * 0.5f;
    error_vx = (sp_FL-FL);
    error_vy = (sp_L - L);

    pid_vx_->input = error_vx; 
    pid_vy_->input = error_vy;

    pid_vx_->setpoint = 0.0f;
    pid_vy_->setpoint = 0.0f;

    pid_vx_->compute();
    pid_vy_->compute();

    geometry_msgs::msg::Twist cmd;
    std_msgs::msg::Float32MultiArray graph_out;

    graph_out.data.clear();
    graph_out.data.push_back(pid_vx_->input);
    graph_out.data.push_back(pid_vy_->input);
    graph_out.data.push_back(pid_vx_->setpoint);
    graph_out.data.push_back(pid_vy_->setpoint);

    cmd.linear.x  = (pid_vx_->output);
    cmd.linear.y  = -(pid_vy_->output);
    //cmd.angular.z = (pid_w_->output);

    RCLCPP_INFO(this->get_logger(),
                 "PID Outputs x: ");
                 
    graph_pub_->publish(graph_out);
    cmd_pub_->publish(cmd);

    setpoint_.data.clear();
    setpoint_.data.push_back(0.0f);
    setpoint_.data.push_back(RESET_YAW_BYTE); 
    setpoint_pub->publish(setpoint_);
}

    // ---------------- ACTION SERVER ----------------
rclcpp_action::GoalResponse PIDServerNode::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Movement::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received PID goal request");

    if (goal_active_) {
        RCLCPP_WARN(this->get_logger(), "PID busy, rejecting new goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

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

//      float step = 1.0f;
//      float error_to_goal = 0.0f;
//      if (goal_active_ && sick_ready_) {
//          if(fabs(true_goal_sp_[0] - sick_vals_[0]) < 5.0f &&
//             fabs(true_goal_sp_[1] - sick_vals_[1]) < 5.0f &&
//             fabs(true_goal_sp_[2] - sick_vals_[2]) < 5.0f &&
//             fabs(true_goal_sp_[3] - sick_vals_[3]) < 5.0f)
//          {
//              for(int i = 0; i < 4; i++){
//                  goal_sp_[i] = true_goal_sp_[i];
//                  printf("Final setpoint given\n");
//              }
//          }
//          else{
//              for(int i = 0; i < 4; i++){
//                  error_to_goal = true_goal_sp_[i] - sick_vals_[i];
//                  goal_sp_[i] += std::copysign(std::fmin(step, fabs(error_to_goal)), error_to_goal) ; // move closer each loop
//                  printf("Intermediate setpoint given\n");
//              }
//          }
//      }
//  }

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

        // Compute errors
        float error_vx = sick_vals_[0] - true_goal_sp_[0];
        float error_vy = sick_vals_[1] - true_goal_sp_[1];

        // Debug print every loop iteration
        RCLCPP_INFO(this->get_logger(),
                    "PID Loop: sick_vals=[%.2f, %.2f], goal_sp=[%.2f, %.2f], error=[%.2f, %.2f],",
                    sick_vals_[0], sick_vals_[1],
                    true_goal_sp_[0], true_goal_sp_[1],
                    error_vx, error_vy);

        // Check if goal is reached
        if (std::fabs(error_vx) < 0.5f && std::fabs(error_vy) < 0.5f) {
            RCLCPP_INFO(this->get_logger(),
                        "PID goal reached! Final Values: sick_vals=[%.2f, %.2f], goal_sp=[%.2f, %.2f], error=[%.2f, %.2f],",
                        sick_vals_[0], sick_vals_[1],
                        true_goal_sp_[0], true_goal_sp_[1],
                        error_vx, error_vy);

            pid_vx_->reset();
            pid_vy_->reset();

            // Stop the robot
            cmd.linear.x = 0.0f;
            cmd.linear.y = 0.0f;
            cmd_pub_->publish(cmd);

            break; // Exit loop
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
