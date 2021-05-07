#ifndef ROBOT_STATE_SHM_H_
#define ROBOT_STATE_SHM_H_

#include <robot_info/robot_macro.h>

namespace robot_state_shm
{
typedef struct
{
    double cur_arm_positions_[ARM_DOF];
    double cur_arm_velocities_[ARM_DOF];
    double cur_arm_efforts_[ARM_DOF];

    double cur_end_eff_positions_[END_EFF_DOF];
    double cur_end_eff_velocities_[END_EFF_DOF];
    double cur_end_eff_efforts_[END_EFF_DOF];
} RobotStateShm;

}

#endif
