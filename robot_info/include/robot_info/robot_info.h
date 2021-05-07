#ifndef ROBOT_INFO_H_
#define ROBOT_INFO_H_

#include <robot_info/arm_info.h>
#ifdef USE_END_EFFECTOR
#include <robot_info/end_eff_info.h>
#endif

namespace robot_info
{
class RobotInfo
{
public:
    RobotInfo();
    ~RobotInfo();

    std::shared_ptr<arm_info::ArmInfo> arm_ = 
        std::make_shared<arm_info::ArmInfo>();

#ifdef USE_END_EFFECTOR
    std::shared_ptr<end_eff_info::EndEffInfo> end_eff_ = 
        std::make_shared<end_eff_info::EndEffInfo>();
#endif

    key_t state_shm_key_;
    key_t state_sem_key_;
};

}

#endif