#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>
#include <process_commu/arm_shm.h>
#include <process_commu/end_eff_shm.h>
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

    rclcpp::WallRate loop_rate(1000);
    unsigned int i = 0;
    while (rclcpp::ok())
    {
        sem_common::semaphore_p(arm_sem_id);
        for (unsigned int j=0; j< robot->arm_->dof_; j++)
        {
            arm_shm->control_modes_[j] = robot->arm_->position_mode_;

            arm_shm->cmd_positions_[j] = arm_shm->cur_positions_[j]+0.001;
            arm_shm->cmd_velocities_[j] = 0;
            arm_shm->cmd_efforts_[j] = 0;
        }
        sem_common::semaphore_v(arm_sem_id);
        loop_rate.sleep();
        if (i++ == 1000)
        {
            RCLCPP_INFO(rclcpp::get_logger("test"), 
                "Execute arm control successfully.");
            break;
        }
    }

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

    i = 0;
    while (rclcpp::ok())
    {
        sem_common::semaphore_p(end_eff_sem_id);
        for (unsigned int j=0; j< robot->end_eff_->dof_; j++)
        {
            end_eff_shm->control_modes_[j] = robot->end_eff_->position_mode_;

            end_eff_shm->cmd_positions_[j] = end_eff_shm->cur_positions_[j]+0.001;
            end_eff_shm->cmd_velocities_[j] = 0;
            end_eff_shm->cmd_efforts_[j] = 0;
        }
        sem_common::semaphore_v(end_eff_sem_id);
        loop_rate.sleep();
        if (i++ == 1000)
        {
            RCLCPP_INFO(rclcpp::get_logger("test"), 
                "Execute end-effector control successfully.");
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}
