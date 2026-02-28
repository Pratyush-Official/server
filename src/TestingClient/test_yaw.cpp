#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cmath>

#define PI 3.14159265358979323846
#define RESET_YAW_BYTE 0x42
#define SET_YAW_BYTE   0x14

class YawSetpointOnce : public rclcpp::Node
{
public:
    YawSetpointOnce() : Node("yaw_setpoint_once")
    {
        imu_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/imu_data", 10,
            std::bind(&YawSetpointOnce::yaw_callback, this, std::placeholders::_1)
        );

        setpoint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "setpoint_yaw", 10
        );

        RCLCPP_INFO(this->get_logger(), "Yaw Setpoint Once node ready");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr setpoint_pub_;

    bool reset_sent_ = false;
    bool set_sent_   = false;

    float wrapAngle(float a)
    {
        while (a > PI)  a -= 2 * PI;
        while (a < -PI) a += 2 * PI;
        return a;
    }

    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        float current_yaw = wrapAngle(msg->data);
        float turn = wrapAngle(current_yaw - PI / 2.0f);
        // float turn = wrapAngle(PI / 2.0f);

        std_msgs::msg::Float32MultiArray out;

        if (!reset_sent_) {
            // Send RESET once
            out.data = {turn, RESET_YAW_BYTE};
            setpoint_pub_->publish(out);
            RCLCPP_INFO(this->get_logger(), "RESET sent: %.2f", turn);
            reset_sent_ = true;

        } else if (!set_sent_) {
            // Send SET once, after RESET has been sent
            out.data = {turn, SET_YAW_BYTE};
            setpoint_pub_->publish(out);
            RCLCPP_INFO(this->get_logger(), "SET sent: %.2f", turn);
            set_sent_ = true;

            // All done, shut down
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawSetpointOnce>());
    return 0;
}
