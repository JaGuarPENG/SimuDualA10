#include "aris_node.hpp"

std::shared_ptr<ArisNode> node_ptr;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  node_ptr = std::make_shared<ArisNode>("aris_node");

  std::thread aris_cmdline_thread(&ArisNode::cmdLineFunction, node_ptr);
  aris_cmdline_thread.detach();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_ptr);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
