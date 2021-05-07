#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>
#include <process_commu/arm_shm.h>
#ifdef USE_END_EFFECTOR
#include <process_commu/end_eff_shm.h>
#endif
#include <robot_info/robot_info.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_info::RobotInfo> robot = 
        std::make_shared<robot_info::RobotInfo>();

    arm_shm::ArmShm *arm_shm;
    int arm_shm_id;

    arm_shm_id = shm_common::create_shm(robot->arm_->shm_key_, &arm_shm);
    if (arm_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Create arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Create arm shared memory failed.");
        return 0;
    }

    int arm_sem_id;
    arm_sem_id = sem_common::create_semaphore(robot->arm_->sem_key_);
    if (arm_sem_id != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Create arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Create arm semaphore failed.");
        return 0;
    }

#ifdef USE_END_EFFECTOR
    ///////////////////////////////////////////////////////////////////////////////////////////////
    end_eff_shm::EndEffShm *end_eff_shm;
    int end_eff_shm_id;

    end_eff_shm_id = shm_common::create_shm(robot->end_eff_->shm_key_, &end_eff_shm);
    if (end_eff_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Create end-effector shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Create end-effector shared memory failed.");
        return 0;
    }

    int end_eff_sem_id;
    end_eff_sem_id = sem_common::create_semaphore(robot->end_eff_->sem_key_);
    if (end_eff_sem_id != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), 
            "Create end-effector semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), 
            "Create end-effector semaphore failed.");
        return 0;
    }
#endif

    rclcpp::WallRate loop_rate(1000);
    while (rclcpp::ok())
    {
        sem_common::semaphore_p(arm_sem_id);
        for (unsigned int j=0; j< robot->arm_->dof_; j++)
        {
            robot->arm_->cur_positions_[j] = arm_shm->cur_positions_[j];
            robot->arm_->cur_velocities_[j] = arm_shm->cur_velocities_[j];
            robot->arm_->cur_efforts_[j] = arm_shm->cur_efforts_[j];

            robot->arm_->cmd_positions_[j] = arm_shm->cmd_positions_[j];
            robot->arm_->cmd_velocities_[j] = arm_shm->cmd_velocities_[j];
            robot->arm_->cmd_efforts_[j] = arm_shm->cmd_efforts_[j];

            robot->arm_->control_modes_[j] = arm_shm->control_modes_[j];

            std::cout << "robot->arm_->cur_positions_[" << j << "]: " << robot->arm_->cur_positions_[j] << std::endl;
            std::cout << "robot->arm_->cur_velocities_[" << j << "]: " << robot->arm_->cur_velocities_[j] << std::endl;
            std::cout << "robot->arm_->cur_efforts_[" << j << "]: " << robot->arm_->cur_efforts_[j] << std::endl;
        
            std::cout << "robot->arm_->cmd_positions_[" << j << "]: " << robot->arm_->cmd_positions_[j] << std::endl;
            std::cout << "robot->arm_->cmd_velocities_[" << j << "]: " << robot->arm_->cmd_velocities_[j] << std::endl;
            std::cout << "robot->arm_->cmd_efforts_[" << j << "]: " << robot->arm_->cmd_efforts_[j] << std::endl;
        
            std::cout << "robot->arm_->control_modes_[" << j << "]: " << robot->arm_->control_modes_[j] << std::endl;
        }
        sem_common::semaphore_v(arm_sem_id);
        std::cout << "-----------------------------" << std::endl;

#ifdef USE_END_EFFECTOR
        /////////////////////////////////////////////////////////////////////////////////////////
        sem_common::semaphore_p(end_eff_sem_id);
        for (unsigned int j=0; j< robot->end_eff_->dof_; j++)
        {
            robot->end_eff_->cur_positions_[j] = end_eff_shm->cur_positions_[j];
            robot->end_eff_->cur_velocities_[j] = end_eff_shm->cur_velocities_[j];
            robot->end_eff_->cur_efforts_[j] = end_eff_shm->cur_efforts_[j];

            robot->end_eff_->cmd_positions_[j] = end_eff_shm->cmd_positions_[j];
            robot->end_eff_->cmd_velocities_[j] = end_eff_shm->cmd_velocities_[j];
            robot->end_eff_->cmd_efforts_[j] = end_eff_shm->cmd_efforts_[j];

            robot->end_eff_->control_modes_[j] = end_eff_shm->control_modes_[j];

            std::cout << "robot->end_eff_->cur_positions_[" << j << "]: " << robot->end_eff_->cur_positions_[j] << std::endl;
            std::cout << "robot->end_eff_->cur_velocities_[" << j << "]: " << robot->end_eff_->cur_velocities_[j] << std::endl;
            std::cout << "robot->end_eff_->cur_efforts_[" << j << "]: " << robot->end_eff_->cur_efforts_[j] << std::endl;
        
            std::cout << "robot->end_eff_->cmd_positions_[" << j << "]: " << robot->end_eff_->cmd_positions_[j] << std::endl;
            std::cout << "robot->end_eff_->cmd_velocities_[" << j << "]: " << robot->end_eff_->cmd_velocities_[j] << std::endl;
            std::cout << "robot->end_eff_->cmd_efforts_[" << j << "]: " << robot->end_eff_->cmd_efforts_[j] << std::endl;
        
            std::cout << "robot->end_eff_->control_modes_[" << j << "]: " << robot->end_eff_->control_modes_[j] << std::endl;
        }
        sem_common::semaphore_v(end_eff_sem_id);
        std::cout << "------------------------------------------------------" << std::endl;
#endif

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
