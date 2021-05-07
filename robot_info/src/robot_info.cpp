#include <robot_info/robot_info.h>
#include <robot_info/robot_macro.h>
#include <sys/sem.h>

namespace robot_info
{
RobotInfo::RobotInfo()
{
    state_shm_key_ = ftok(ROBOT_STATE_SHM_FILE, 1);
    if (state_shm_key_ == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("robot_info"),
            "Generate key value of robot state shared memory failed.");
    }

    state_sem_key_ = ftok(ROBOT_STATE_SEM_FILE, 1);
    if (state_sem_key_ == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("robot_info"),
            "Generate key value of robot state semaphore failed.");
    }
}

RobotInfo::~RobotInfo(){}

}