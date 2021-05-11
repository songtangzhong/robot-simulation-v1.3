#include <rclcpp/rclcpp.hpp>
#include <robot_fun/robot_fun.h>
#include <robot_info/robot_macro.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_fun::RobotFun> robot = 
        std::make_shared<robot_fun::RobotFun>("test");

    double cur_arm_positions[ARM_DOF];
    std::vector<double> cur_arm_positions_1;
    std::vector<double> cur_arm_positions_2;
    cur_arm_positions_1.resize(ARM_DOF);
    cur_arm_positions_2.resize(6);

    while (rclcpp::ok())
    {
        robot->get_arm_joint_positions(cur_arm_positions);
        for (unsigned int j=0; j< ARM_DOF; j++)
        {
            cur_arm_positions_1[j] = cur_arm_positions[j]+0.5;
            cur_arm_positions_2[j] = cur_arm_positions[j]+1;
        }

        robot->set_arm_joint_positions(cur_arm_positions_1);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        robot->set_arm_joint_positions(cur_arm_positions_2);

        break;
    }

    rclcpp::shutdown();
    return 0;
}
