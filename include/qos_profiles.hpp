#ifndef QOS_PROFILES_HPP
#define QOS_PROFILES_HPP


#pragma once
#include <rclcpp/rclcpp.hpp>

inline rclcpp::QoS sensor_qos(){
    return(rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile());

}

inline rclcpp::QoS cmd_vel_qos(){
    return(rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    
}

inline rclcpp::QoS state_qos(){
    return(rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
    
}
#endif