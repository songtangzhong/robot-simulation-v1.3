#include <rclcpp/rclcpp.hpp>
#include <robot_fun/robot_fun.h>

int main(int argc, char ** argv)
{
    if (argc != 2)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_switch_to_target_controller"), 
            "Usage: ros2 run robot_fun test_switch_to_target_controller position_controllers");

        return 0;
    }

    rclcpp::init(argc, argv);

    std::shared_ptr<robot_fun::RobotFun> robot = 
        std::make_shared<robot_fun::RobotFun>("test_switch_to_target_controller");

    rclcpp::WallRate wait_ready(1);
    unsigned int count = 5;
    for (unsigned int j=0; j<=count; j++)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_switch_to_target_controller"), 
            "Waitting for the operating environment to be ready... %d seconds.", count-j);
        wait_ready.sleep();
    }

    std::string target_controller = argv[1];
    if (robot->arm_switch_controller(target_controller) != -1)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_switch_to_target_controller"), 
            "Switch to %s successfully.", target_controller.c_str());
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("test_switch_to_target_controller"), 
            "Switch to %s failed.", target_controller.c_str());
    }

    rclcpp::shutdown();
    return 0;
}