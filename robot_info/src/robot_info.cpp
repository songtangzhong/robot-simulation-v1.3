#include <robot_info/robot_info.h>
#include <robot_info/robot_macro.h>

namespace robot_info
{
RobotInfo::RobotInfo()
{
    state_shm_key_ = ROBOT_STATE_SHM_KEY;
    state_sem_key_ = ROBOT_STATE_SEM_KEY;
}

RobotInfo::~RobotInfo(){}

}