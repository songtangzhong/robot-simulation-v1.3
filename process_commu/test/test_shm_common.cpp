#include <process_commu/shm_common.h>
#include <process_commu/arm_shm.h>
#include <robot_info/arm_info.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<arm_info::ArmInfo> arm = 
        std::make_shared<arm_info::ArmInfo>();
    arm_shm::ArmShm *arm_shm;
    int arm_shm_id;

    arm_shm_id = shm_common::create_shm(arm->shm_key_, &arm_shm);
    if (arm_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), "Create arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), "Create arm shared memory failed.");
        return 0;
    }

    for (unsigned int j=0; j< arm->dof_; j++)
    {
        arm_shm->cur_positions_[j] = 1;
        arm_shm->cur_velocities_[j] = 1;
        arm_shm->cur_efforts_[j] = 1;

        arm_shm->cmd_positions_[j] = 1;
        arm_shm->cmd_velocities_[j] = 1;
        arm_shm->cmd_efforts_[j] = 1;

        arm_shm->control_modes_[j] = 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    for (unsigned int j=0; j< arm->dof_; j++)
    {
        arm->cur_positions_[j] = arm_shm->cur_positions_[j];
        arm->cur_velocities_[j] = arm_shm->cur_velocities_[j];
        arm->cur_efforts_[j] = arm_shm->cur_efforts_[j];

        arm->cmd_positions_[j] = arm_shm->cmd_positions_[j];
        arm->cmd_velocities_[j] = arm_shm->cmd_velocities_[j];
        arm->cmd_efforts_[j] = arm_shm->cmd_efforts_[j];

        arm->control_modes_[j] = arm_shm->control_modes_[j];

        std::cout << "arm->cur_positions_[" << j << "]: " << arm->cur_positions_[j] << std::endl;
        std::cout << "arm->cur_velocities_[" << j << "]: " << arm->cur_velocities_[j] << std::endl;
        std::cout << "arm->cur_efforts_[" << j << "]: " << arm->cur_efforts_[j] << std::endl;
    
        std::cout << "arm->cmd_positions_[" << j << "]: " << arm->cmd_positions_[j] << std::endl;
        std::cout << "arm->cmd_velocities_[" << j << "]: " << arm->cmd_velocities_[j] << std::endl;
        std::cout << "arm->cmd_efforts_[" << j << "]: " << arm->cmd_efforts_[j] << std::endl;
    
        std::cout << "arm->control_modes_[" << j << "]: " << arm->control_modes_[j] << std::endl;
    }

    if (shm_common::release_shm(arm_shm_id, &arm_shm) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), "Release arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), "Release arm shared memory failed.");
        return 0;
    }
    
    rclcpp::shutdown();
    return 0;
}
