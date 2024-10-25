#include "aris_node.hpp"
#include "robot.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

ArisNode::ArisNode(std::string name) : Node(name), cs_(aris::server::ControlServer::instance()) {
    initRos2();

    auto xmlpath = ament_index_cpp::get_package_share_directory("a10_dual") + "/xml/cs.xml";
    aris::core::fromXmlFile(cs_, xmlpath);

    cs_.init();

    cs_.open();

    cs_.start();

    std::cout << "Control Server Init" << std::endl;
}

void ArisNode::initRos2() {
    callback_group_00_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_pub_joint_state_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() {
            for (int i = 0; i < 12; i++) {
                joint_pos_fdb_[i] = this->cs_.controller().motorPool()[i].actualPos();
                joint_vel_fdb_[i] = this->cs_.controller().motorPool()[i].actualVel();
                joint_tor_fdb_[i] = this->cs_.controller().motorPool()[i].actualToq();
            }

            msg_joint_state_.header.stamp = this->now();
            msg_joint_state_.name.clear();
            msg_joint_state_.position.clear();
            msg_joint_state_.velocity.clear();
            msg_joint_state_.effort.clear();

            for (int i = 0; i < 6; i++) {
                msg_joint_state_.name.push_back("Arm1_joint" + std::to_string(i + 1));
                msg_joint_state_.position.push_back(joint_pos_fdb_[i]);
                msg_joint_state_.velocity.push_back(joint_vel_fdb_[i]);
                msg_joint_state_.effort.push_back(joint_tor_fdb_[i]);
            }
            for (int i = 6; i < 12; i++) {
                msg_joint_state_.name.push_back("Arm2_joint" + std::to_string(i + 1 - 6));
                msg_joint_state_.position.push_back(joint_pos_fdb_[i]);
                msg_joint_state_.velocity.push_back(joint_vel_fdb_[i]);
                msg_joint_state_.effort.push_back(joint_tor_fdb_[i]);
            }
            publisher_joint_state_->publish(msg_joint_state_);
        },
        callback_group_00_
    );
}

void ArisNode::cmdLineFunction() {
    cs_.runCmdLine();
}
