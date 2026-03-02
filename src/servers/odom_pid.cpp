#include "servers/odom_pid.hpp"
#include "qos_profiles.hpp"

OdomPIDServer::OdomPIDServer()
    : Node("pid_server_node")
    {

        this->declare_parameter<double>("odom.vx.kp", 2.0);
        this->declare_parameter<double>("odom.vx.ki", 0.0);
        this->declare_parameter<double>("odom.vx.kd", 0.001);

        this->declare_parameter<double>("odom.vy.kp", 0.1);
        this->declare_parameter<double>("odom.vy.ki", 0.1);
        this->declare_parameter<double>("odom.vy.kd", 0.0);

        // this->declare_parameter<double>("pid.w.kp",  0.01);
        // this->declare_parameter<double>("pid.w.ki",  0.1);
        // this->declare_parameter<double>("pid.w.kd",  0.0);

        pid_vx_ = std::make_unique<PID>(
            this->get_parameter("odom.vx.kp").as_double(),
            this->get_parameter("odom.vx.ki").as_double(),
            this->get_parameter("odom.vx.kd").as_double(),
            -0.2f, 0.2f, 10, 0.01f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);
        

        pid_vy_ = std::make_unique<PID>(
            this->get_parameter("odom.vy.kp").as_double(),
            this->get_parameter("odom.vy.ki").as_double(),
            this->get_parameter("odom.vy.kd").as_double(),
            -0.2f, 0.2f, 10, 0.01f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());

        // graph_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pid_tune", 10);

        // slider_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //     "/pid_gains", 10,
        //     std::bind(&OdomPIDServer::sliderCallback, this, std::placeholders::_1));

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



        RCLCPP_INFO(this->get_logger(), "PID mecanum server started");
    }

    // ---------------- CALLBACKS ----------------
void OdomPIDServer::odomCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 2) {
        data = msg->data;

        const float wheel_diameter = 0.0574f; 
        const int counts_per_rev = 4000;

        float dist1 = (data[0] * 3.1416f * wheel_diameter) / counts_per_rev;
        float dist2 = (data[1] * 3.1416f * wheel_diameter) / counts_per_rev;

        odom_vals_[0] = dist1; 
        odom_vals_[1] = dist2;

        odom_ready_ = true;
    }
}

// void OdomPIDServer::sliderCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
// {
//     if (msg->data.size() >= 4) {
//         slider_data_ = msg->data;
//         printf("Slider data: ");
//         printf("%.3f ", slider_data_[0]);
//         printf("\n");
//     }
// }

void OdomPIDServer::pidLoop()
{

    RCLCPP_DEBUG(this->get_logger(), "PID Loop entered");
    // if (!pid_vx_) return;
    if (!odom_ready_ || !goal_active_) return;

    pid_vx_->setTunings(2.0f, 0.0f, 0.0f);

    float F = odom_vals_[0];
    float S  = odom_vals_[1];

    float sp_F = true_goal_sp_[0];
    float sp_S  = true_goal_sp_[1];

    error_vx = (sp_F - F);
    error_vy = (sp_S - S);

    pid_vx_->input = error_vx; 
    pid_vy_->input = error_vy;

    pid_vx_->setpoint = 0.0f;
    pid_vy_->setpoint = 0.0f;

    pid_vx_->compute();
    pid_vy_->compute();

    geometry_msgs::msg::Twist cmd;
    std_msgs::msg::Float32MultiArray graph_out;

    // graph_out.data.clear();
    // graph_out.data.push_back(pid_vx_->input);
    // graph_out.data.push_back(pid_vy_->input);
    // graph_out.data.push_back(pid_vx_->setpoint);
    // graph_out.data.push_back(pid_vy_->setpoint);

    cmd.linear.x  = (pid_vx_->output);
    // cmd.linear.y  = -(pid_vy_->output);
    //cmd.angular.z = (pid_w_->output);

    RCLCPP_INFO(this->get_logger(),
                 "PID Outputs x: %.2f, y: %.2f", pid_vx_->output, pid_vy_->output);
                 
    // graph_pub_->publish(graph_out);
    cmd_pub_->publish(cmd);

    setpoint_.data.clear();
    setpoint_.data.push_back(0.0f);
    setpoint_.data.push_back(RESET_YAW_BYTE); 
    setpoint_pub->publish(setpoint_);
}

    // ---------------- ACTION SERVER ----------------
rclcpp_action::GoalResponse OdomPIDServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Movement::Goal> goal)
{
    (void) uuid; // silence unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Received PID goal request");

    if (goal_active_) {
        RCLCPP_WARN(this->get_logger(), "PID busy, rejecting new goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->setpoint_odom.size() >= 2) {
        true_goal_sp_.resize(2);  // ensure size
        for (size_t i = 0; i < 2; ++i) {
            true_goal_sp_[i] = static_cast<float>(goal->setpoint_odom[i]); // convert mm -> m if needed
        }

        goal_active_ = true;

        RCLCPP_INFO(this->get_logger(), "PID goal received and setpoints updated to : %.2f, %.2f", true_goal_sp_[0], true_goal_sp_[1]);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RCLCPP_WARN(this->get_logger(), "PID goal rejected: setpoint_odom has less than 2 elements");
        return rclcpp_action::GoalResponse::REJECT;
    }
}

// void OdomPIDServer::trajectory_tracking(){

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

rclcpp_action::CancelResponse OdomPIDServer::handle_cancel(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    (void) goal_handle; // silence unused parameter warning
    goal_active_ = false;
    RCLCPP_WARN(this->get_logger(), "PID goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OdomPIDServer::handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting PID goal execution");
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void OdomPIDServer::execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    auto result = std::make_shared<Movement::Result>();
    geometry_msgs::msg::Twist cmd;
    rclcpp::Rate rate(100); // 10ms loop
    //const float tol = 0.5f;
    while (rclcpp::ok() && goal_active_ && odom_ready_) {
        pidLoop();

        // Compute errors
        float error_vx = odom_vals_[0] - true_goal_sp_[0];
        float error_vy = odom_vals_[1] - true_goal_sp_[1];

        // Debug print every loop iteration
        RCLCPP_INFO(this->get_logger(),
                    "PID Loop: odom_vals=[%.2f, %.2f], goal_sp=[%.2f, %.2f], error=[%.2f, %.2f],",
                    odom_vals_[0], odom_vals_[1],
                    true_goal_sp_[0], true_goal_sp_[1],
                    error_vx, error_vy);

        // Check if goal is reached
        if (std::fabs(error_vx) < 0.5f) {
            RCLCPP_INFO(this->get_logger(),
                        "PID goal reached! Final Values: odom_vals=[%.2f, %.2f], goal_sp=[%.2f, %.2f], error=[%.2f, %.2f],",
                        odom_vals_[0], odom_vals_[1],
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
    rclcpp::spin(std::make_shared<OdomPIDServer>());
    rclcpp::shutdown();
    return 0;
}
