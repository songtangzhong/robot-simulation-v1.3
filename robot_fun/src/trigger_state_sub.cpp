#include <rclcpp/rclcpp.hpp>
#include <robot_fun/robot_fun.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_fun::RobotFun> node = 
        std::make_shared<robot_fun::RobotFun>("trigger_state_sub");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
