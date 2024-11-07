#include "dwa_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DWANode>();
  try
  {
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    // エラーハンドリング
  }
  rclcpp::shutdown();
  return 0;
}