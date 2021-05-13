#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/controller_configure.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3)
  {
      RCLCPP_INFO(rclcpp::get_logger("switch_controller"), 
      "Usage: ros2 run robot_hw_interface test_switch_controller start_controller stop_controller");

      return 0;
  }

  std::string start_controller = argv[1];
  std::string stop_controller = argv[2];
  std::shared_ptr<controller_configure::ControllerConfigure> manager = 
    std::make_shared<controller_configure::ControllerConfigure>("test_switch_controller");

  rclcpp::WallRate wait_ready(1);
  unsigned int count = 5;
  for (unsigned int j=0; j<=count; j++)
  {
    RCLCPP_INFO(rclcpp::get_logger("test_switch_controller"), 
      "Waitting for the operating environment to be ready... %d seconds.", count-j);
    wait_ready.sleep();
  }

  manager->switch_controller(start_controller, stop_controller);
  
  rclcpp::shutdown();
  return 0;
}
