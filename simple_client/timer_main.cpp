#include "rclcpp/rclcpp.hpp"

#include "timer_client_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<TimerClientNode> node = std::make_shared<TimerClientNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;

}
