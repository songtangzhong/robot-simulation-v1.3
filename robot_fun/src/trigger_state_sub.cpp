#include <rclcpp/rclcpp.hpp>
#include <robot_fun/robot_fun.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_fun::RobotFun> node = 
        std::make_shared<robot_fun::RobotFun>("trigger_state_sub");

    rclcpp::WallRate loop_rate(1);
    unsigned int count = 5;
    for (unsigned int j=0; j<=count; j++)
    {
        RCLCPP_INFO(rclcpp::get_logger("trigger_state_sub"), 
            "Waitting for the operating environment to be ready... %d seconds.", count-j);
        loop_rate.sleep();
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
