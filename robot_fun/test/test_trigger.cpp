#include <rclcpp/rclcpp.hpp>
#include <robot_fun/robot_fun.h>
#include <robot_info/robot_macro.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_fun::RobotFun> robot = 
        std::make_shared<robot_fun::RobotFun>("test_trigger");

    rclcpp::WallRate loop_rate(1);
    double cur_arm_positions[ARM_DOF];
    double cur_arm_velocities[ARM_DOF];
    double cur_arm_efforts[ARM_DOF];

    while (rclcpp::ok())
    {
        robot->get_arm_joint_positions(cur_arm_positions);
        robot->get_arm_joint_velocities(cur_arm_velocities);
        robot->get_arm_joint_efforts(cur_arm_efforts);
        
        for (unsigned int j=0; j< ARM_DOF; j++)
        {
            std::cout << "cur_positions[" << j << "]: " << cur_arm_positions[j] << std::endl;
        }
        for (unsigned int j=0; j< ARM_DOF; j++)
        {
            std::cout << "cur_velocities[" << j << "]: " << cur_arm_velocities[j] << std::endl;
        }
        for (unsigned int j=0; j< ARM_DOF; j++)
        {
            std::cout << "cur_efforts[" << j << "]: " << cur_arm_efforts[j] << std::endl;
        }
        std::cout << "-------------------------------" << std::endl;
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
