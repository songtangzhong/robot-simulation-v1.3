#include <process_commu/shm_common.h>
#include <process_commu/arm_shm.h>
#include <process_commu/end_eff_shm.h>
#include <process_commu/robot_state_shm.h>
#include <rclcpp/rclcpp.hpp>

namespace shm_common
{
template <class T>
int create_shm(key_t key, T ** shm_ptr)
{
    void *shm_ln_ptr = NULL;

    T *shm_ptr_;

    int shm_id;

    shm_id = shmget(key, sizeof(T), IPC_CREAT | 0666);
    if (shm_id == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("shm_common"), "Create shared memory failed.");
		return SHM_STATE_NO;
    }

    shm_ln_ptr = shmat(shm_id, 0, 0);
    if (shm_ln_ptr == (void*)-1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("shm_common"), "Create link address failed.");
		return SHM_STATE_NO;
    }

    *shm_ptr = shm_ptr_ = (T*)shm_ln_ptr;

	return shm_id;
}

template <class T>
int release_shm(int shm_id, T ** shm_ptr)
{
    if (shmdt(*shm_ptr) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("shm_common"), "Seperate shared memory failed.");
		return SHM_STATE_NO;
    }

    if (shmctl(shm_id, IPC_RMID, 0) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("shm_common"), "Release shared memory failed.");
		return SHM_STATE_NO;
    }

	return SHM_STATE_OK;
}

template int create_shm<arm_shm::ArmShm>(key_t key, arm_shm::ArmShm ** shm_ptr);
template int release_shm<arm_shm::ArmShm>(int shm_id, arm_shm::ArmShm ** shm_ptr);

template int create_shm<end_eff_shm::EndEffShm>(key_t key, end_eff_shm::EndEffShm ** shm_ptr);
template int release_shm<end_eff_shm::EndEffShm>(int shm_id, end_eff_shm::EndEffShm ** shm_ptr);

template int create_shm<robot_state_shm::RobotStateShm>(key_t key, robot_state_shm::RobotStateShm ** shm_ptr);
template int release_shm<robot_state_shm::RobotStateShm>(int shm_id, robot_state_shm::RobotStateShm ** shm_ptr);

}
