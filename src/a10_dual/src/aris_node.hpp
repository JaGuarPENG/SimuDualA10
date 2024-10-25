#ifndef ARIS_NODE_HPP_
#define ARIS_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "aris.hpp"

class ArisNode : public rclcpp::Node {
public:
    ArisNode(std::string name);
    void initRos2();
    void cmdLineFunction();

    aris::server::ControlServer& cs_;

    double joint_pos_fdb_[12];
    double joint_vel_fdb_[12];
    // double joint_acc_fdb_[6];
    double joint_tor_fdb_[12];
    // double ee_pos_fdb_[6];
    // double ee_vel_fdb_[6];
    // double ee_acc_fdb_[6];


    // std_msgs::msg::String msg_string_;
    sensor_msgs::msg::JointState msg_joint_state_;

    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_string_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_state_;

    // rclcpp::TimerBase::SharedPtr timer_pub_string_;
    rclcpp::TimerBase::SharedPtr timer_pub_joint_state_;
    // rclcpp::TimerBase::SharedPtr time01_;
    // rclcpp::TimerBase::SharedPtr time02_;

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_00_;
    // rclcpp::CallbackGroup::SharedPtr callback_group_01_;
    // rclcpp::CallbackGroup::SharedPtr callback_group_02_;
    // rclcpp::CallbackGroup::SharedPtr callback_group_03_;

    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_execute_string_;
};

#endif

