#include <rclcpp/rclcpp.hpp>
#include <robot_fun/robot_fun.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_fun::RobotFun> robot = 
        std::make_shared<robot_fun::RobotFun>("test_get_arm_control_mode");

    rclcpp::WallRate wait_ready(1);
    unsigned int count = 5;
    for (unsigned int j=0; j<=count; j++)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_get_arm_control_mode"), 
            "Waitting for the operating environment to be ready... %d seconds.", count-j);
        wait_ready.sleep();
    }

    std::string control_mode = robot->get_arm_control_mode();
    std::cout << "current arm control mode: " << control_mode << std::endl;

    rclcpp::shutdown();
    return 0;
}