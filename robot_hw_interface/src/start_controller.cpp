#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/controller_configure.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<controller_configure::ControllerConfigure> manager = 
    std::make_shared<controller_configure::ControllerConfigure>("start_controller");

  manager->load_start_controller("joint_state_controller");
  manager->load_start_controller("position_controllers");
  manager->load_configure_controller("velocity_controllers");
  manager->load_configure_controller("effort_controllers");

  rclcpp::shutdown();
  return 0;
}
