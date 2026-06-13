#include "servers/sick_pid_server.hpp"
#include "qos_profiles.hpp"
#include <fstream>
#include <chrono>

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
            -0.2f, 0.2, 10, 2.0f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

        pid_vy_ = std::make_unique<PID>(
            this->get_parameter("pid.vy.kp").as_double(),
            this->get_parameter("pid.vy.ki").as_double(),
            this->get_parameter("pid.vy.kd").as_double(),
            -0.2f, 0.2, 10, 2.0f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

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

        pid_vx_->setTunings(0.01f, 0.0f, 0.0f); //0001=kd
        pid_vy_->setTunings(0.01f, 0.0f, 0.0f); //0001=kd

        pid_vx_->init();
        pid_vy_->init();

        // --- CSV logging setup ---
        csv_file_.open("/home/maomao/ROBOCON_26/2026_ros2_ws/src/pid_log.csv");
        csv_file_ << "timestamp,vx_input,vx_setpoint,vx_output,vy_input,vy_setpoint,vy_output\n";

        RCLCPP_INFO(this->get_logger(), "Sick PID Server started");
    }

SickPIDServer::~SickPIDServer()
{
    if (csv_file_.is_open()) csv_file_.close();
}


// ---------------- CALLBACKS ----------------
void SickPIDServer::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data.size() >= 4) {
        if (!filter_initialized_) {
            sick_vals_ = msg->data;
            filtered_FL_ = msg->data[0];
            filtered_L_  = msg->data[1];
            filter_initialized_ = true;
        } else {
            float alpha = 0.15f;
            filtered_FL_ = filtered_FL_ + alpha * (msg->data[0] - filtered_FL_);
            filtered_L_  = filtered_L_  + alpha * (msg->data[1] - filtered_L_);
            sick_vals_[0] = filtered_FL_;
            sick_vals_[1] = filtered_L_;
            sick_vals_[2] = msg->data[2];
            sick_vals_[3] = msg->data[3];
        }
        sick_ready_ = true;
    }
}

// void SickPIDServer::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
// {
//     std::lock_guard<std::mutex> lock(data_mutex_);
//     if (msg->data.size() >= 4) {
//         sick_vals_ = msg->data;
//         sick_ready_ = true;
//     }
// }

void SickPIDServer::pidLoop()
{
    geometry_msgs::msg::Twist cmd;
    std_msgs::msg::Float32MultiArray graph_out;
    RCLCPP_DEBUG(this->get_logger(), "PID Loop entered");

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!sick_ready_ || !goal_active_ || true_goal_sp_.size() < 4) return;

        FL = sick_vals_[0];
        L  = sick_vals_[1];
        sp_FL = true_goal_sp_[0];
        sp_L  = true_goal_sp_[1];
    }

    error_vx = (sp_FL - FL);
    error_vy = (sp_L  - L);

    float deadband = 5.0f; // This is basically to ignore tiny changes in the sick data that would cause the robot to jitter. 

    if (std::abs(sp_FL - FL) < deadband) {
        pid_vx_->input = sp_FL;  // lie to the PID, tell it error is zero
    } else {
        pid_vx_->input = FL;
    }

    if (std::abs(sp_L - L) < deadband) {
        pid_vy_->input = sp_L;
    } else {
        pid_vy_->input = L;
    }
    pid_vx_->setpoint = sp_FL;
    pid_vy_->setpoint = sp_L;

    pid_vx_->compute();
    pid_vy_->compute();

    // --- CSV logging ---
    double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    csv_file_ << t                 << ","
              << pid_vx_->input    << "," << pid_vx_->setpoint << "," << pid_vx_->output << ","
              << pid_vy_->input    << "," << pid_vy_->setpoint << "," << pid_vy_->output << "\n";

    if (++csv_write_count_ % 100 == 0) csv_file_.flush();
    // -------------------

    cmd.linear.x = -(pid_vx_->output);
    cmd.linear.y =  (pid_vy_->output);

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
        true_goal_sp_.resize(4);
        for (size_t i = 0; i < 4; ++i) {
            true_goal_sp_[i] = static_cast<float>(goal->setpoint_sick[i]);
        }
        goal_active_ = true;
        resetVel();
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RCLCPP_WARN(this->get_logger(), "PID goal rejected: setpoint_sick has less than 4 elements");
        resetVel();
        return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse SickPIDServer::handle_cancel(
    const std::shared_ptr<GoalHandleMovement> goal_handle)
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
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void SickPIDServer::execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
{
    auto result = std::make_shared<Movement::Result>();
    rclcpp::Rate rate(100); // 10ms loop

    if (goal_handle->is_canceling())

        {
            geometry_msgs::msg::Twist stop;
            stop.linear.x = 0.0f;
            stop.linear.y = 0.0f;
            cmd_pub_->publish(stop);

            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "[OdomPIDServer] Goal cancelled – stopping.");
            return;
        }

    while (rclcpp::ok() && goal_active_ && sick_ready_) {

        if (pid_vx_->is_allowable_error && pid_vy_->is_allowable_error)  {
                goal_active_ = false;
                result->success = true;

                RCLCPP_INFO(this->get_logger(), "PID goal reached");

                resetVel();
                pid_vx_->reset();
                pid_vy_->reset();
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