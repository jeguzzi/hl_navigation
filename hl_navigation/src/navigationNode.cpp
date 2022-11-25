/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "Navigator.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Navigator>();
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("hl_navigation"), "Shutting down");
  rclcpp::shutdown();
  return 0;
}
