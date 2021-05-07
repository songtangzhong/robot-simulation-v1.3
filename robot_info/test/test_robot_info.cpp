#include <robot_info/robot_info.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_info::RobotInfo> robot = 
        std::make_shared<robot_info::RobotInfo>();

    std::cout << "robot info:" << std::endl;
    std::cout << "state_shm_key: " << robot->state_shm_key_ << std::endl;
    std::cout << "state_sem_key: " << robot->state_sem_key_ << std::endl;

    std::cout << "arm info:" << std::endl;
    std::cout << "shm_key: " << robot->arm_->shm_key_ << std::endl;
    std::cout << "sem_key: " << robot->arm_->sem_key_ << std::endl;
    std::cout << "dof: " << robot->arm_->dof_ << std::endl;
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cur_positions[" << j << "]: " << robot->arm_->cur_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cur_velocities[" << j << "]: " << robot->arm_->cur_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cur_efforts[" << j << "]: " << robot->arm_->cur_efforts_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cmd_positions[" << j << "]: " << robot->arm_->cmd_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cmd_velocities[" << j << "]: " << robot->arm_->cmd_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "cmd_efforts[" << j << "]: " << robot->arm_->cmd_efforts_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        std::cout << "control_modes[" << j << "]: " << robot->arm_->control_modes_[j] << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "end-effector info:" << std::endl;
    std::cout << "shm_key: " << robot->end_eff_->shm_key_ << std::endl;
    std::cout << "sem_key: " << robot->end_eff_->sem_key_ << std::endl;
    std::cout << "dof: " << robot->end_eff_->dof_ << std::endl;
    for (unsigned int j=0; j< robot->end_eff_->dof_; j++)
    {
        std::cout << "cur_positions[" << j << "]: " << robot->end_eff_->cur_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->end_eff_->dof_; j++)
    {
        std::cout << "cur_velocities[" << j << "]: " << robot->end_eff_->cur_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->end_eff_->dof_; j++)
    {
        std::cout << "cur_efforts[" << j << "]: " << robot->end_eff_->cur_efforts_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->end_eff_->dof_; j++)
    {
        std::cout << "cmd_positions[" << j << "]: " << robot->end_eff_->cmd_positions_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->end_eff_->dof_; j++)
    {
        std::cout << "cmd_velocities[" << j << "]: " << robot->end_eff_->cmd_velocities_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->end_eff_->dof_; j++)
    {
        std::cout << "cmd_efforts[" << j << "]: " << robot->end_eff_->cmd_efforts_[j] << std::endl;
    }
    for (unsigned int j=0; j< robot->end_eff_->dof_; j++)
    {
        std::cout << "control_modes[" << j << "]: " << robot->end_eff_->control_modes_[j] << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
