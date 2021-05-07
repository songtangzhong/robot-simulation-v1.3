#include <process_commu/sem_common.h>
#include <rclcpp/rclcpp.hpp>

namespace sem_common
{
int create_semaphore(key_t key)
{
    int sem_id;

    sem_id = semget(key, 1, IPC_CREAT | 0666);
    if (sem_id == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("sem_common"), "Create semaphore failed.");
		return SEM_STATE_NO;
    }

	sem_un sem;
    sem.val = 1;
    if  (semctl(sem_id, 0, SETVAL, sem) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("sem_common"), "Init semaphore failed.");
        return SEM_STATE_NO;
    }

    return sem_id;
}

int delete_semaphore(int sem_id)
{
	sem_un sem;

    if (semctl(sem_id, 0, IPC_RMID, sem) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("sem_common"), "Delete semaphore failed.");
        return SEM_STATE_NO;
    }

    return SEM_STATE_OK;
}

int semaphore_p(int sem_id)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = -1;
	sem_b.sem_flg = SEM_UNDO;
	if (semop(sem_id, &sem_b, 1) == -1)
	{
		RCLCPP_ERROR(rclcpp::get_logger("sem_common"), "Get semaphore failed.");
		return SEM_STATE_NO;
	}

    return SEM_STATE_OK;
}

int semaphore_v(int sem_id)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = 1;
	sem_b.sem_flg = SEM_UNDO;
	if (semop(sem_id, &sem_b, 1) == -1)
	{
		RCLCPP_ERROR(rclcpp::get_logger("sem_common"), "Release semaphore failed.");
		return SEM_STATE_NO;
	}

	return SEM_STATE_OK;
}

}
