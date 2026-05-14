#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robotlibpc/cpp/controller/pid.hpp"
#include "qos_profiles.hpp"
#include <mutex>
#include <memory>

class SickPIDNode : public rclcpp::Node
{
public:
    SickPIDNode()
    : Node("sick_pid_node")
    {
        // Declare PID parameters
        this->declare_parameter<double>("pid.vx.kp", 0.01);
        this->declare_parameter<double>("pid.vx.ki", 0.0);
        this->declare_parameter<double>("pid.vx.kd", 0.0);

        this->declare_parameter<double>("pid.vy.kp", 0.009);
        this->declare_parameter<double>("pid.vy.ki", 0.0);
        this->declare_parameter<double>("pid.vy.kd", 0.0);

        pid_vx_ = std::make_unique<PID>(
            this->get_parameter("pid.vx.kp").as_double(),
            this->get_parameter("pid.vx.ki").as_double(),
            this->get_parameter("pid.vx.kd").as_double(),
            -0.5f, 0.5f, 10, 2.0f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

        pid_vy_ = std::make_unique<PID>(
            this->get_parameter("pid.vy.kp").as_double(),
            this->get_parameter("pid.vy.ki").as_double(),
            this->get_parameter("pid.vy.kd").as_double(),
            -0.5f, 0.5f, 10, 2.0f, DIRECT, PROPORTIONAL_ON_ERROR, AUTOMATIC);

        pid_vx_->init();
        pid_vy_->init();

        // Subscribers
        sick_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sick_data", sensor_qos(),
            std::bind(&SickPIDNode::sickCallback, this, std::placeholders::_1));

        slider_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pid_gains", 10,
            std::bind(&SickPIDNode::sliderCallback, this, std::placeholders::_1));

        // Publishers
        cmd_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
        graph_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sick_error", 10);

        // Control loop timer at 100 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SickPIDNode::pidLoop, this));

        RCLCPP_INFO(this->get_logger(), "Sick PID Node started");
    }

private:

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sick_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr slider_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr graph_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<PID> pid_vx_;
    std::unique_ptr<PID> pid_vy_;


    std::mutex data_mutex_;
    std::vector<float> sick_vals_;
    bool sick_ready_ = false;

    // Setpoints — written by slider, read by pidLoop
    float sp_FL_ = 0.0f;
    float sp_L_  = 0.0f;

    void sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg->data.size() >= 4) {
            sick_vals_ = msg->data;
            sick_ready_ = true;
        }
    }

    void sliderCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Slider message too short (need at least 2 elements)");
            return;
        }

        sp_FL_ = msg->data[0];
        sp_L_  = msg->data[1];

        // live gain tuning from slider
        if (msg->data.size() >= 5) {
            pid_vx_->setTunings(msg->data[2], msg->data[3], msg->data[4]);
        }
        if (msg->data.size() >= 8) {
            pid_vy_->setTunings(msg->data[5], msg->data[6], msg->data[7]);
        }

        RCLCPP_DEBUG(this->get_logger(), "Setpoints updated — sp_FL: %.3f  sp_L: %.3f x_kp: %.3f x_ki: %.3f x_kd: %.3f",
                     sp_FL_, sp_L_, msg->data[2], msg->data[3], msg->data[4]);
    }

    void pidLoop()
    {
        float FL, L, sp_FL, sp_L;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!sick_ready_) return;

            FL = sick_vals_[0];
            L = sick_vals_[1];
            sp_FL = sp_FL_;
            sp_L = sp_L_;
        }

        pid_vx_->input = FL;
        pid_vx_->setpoint = sp_FL;
        pid_vx_->compute();

        pid_vy_->input = L;
        pid_vy_->setpoint = sp_L;
        pid_vy_->compute();

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -(pid_vx_->output);
        cmd.linear.y = (pid_vy_->output);
        cmd_pub_->publish(cmd);

        // Publish diagnostics for PlotJuggler / rqt
        std_msgs::msg::Float32MultiArray graph_out;
        graph_out.data = {
            pid_vx_->input,
            pid_vx_->setpoint,
            pid_vx_->output,
            pid_vy_->input,
            pid_vy_->setpoint,
            pid_vy_->output
        };
        graph_pub_->publish(graph_out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SickPIDNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}