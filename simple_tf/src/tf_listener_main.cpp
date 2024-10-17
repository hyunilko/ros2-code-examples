#include "rclcpp/rclcpp.hpp"
#include "simple_tf/simple_tf_listener_node.hpp"

int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a shared pointer to the SimpleTFListenerNode
  auto node = std::make_shared<SimpleTFListenerNode>();

  // Spin the node to process callbacks
  rclcpp::spin(node);

  // Shutdown ROS 2 when finished
  rclcpp::shutdown();
  
  return 0;
}
