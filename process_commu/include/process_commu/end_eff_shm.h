#ifndef END_EFF_SHM_H_
#define END_EFF_SHM_H_

#include <robot_info/robot_macro.h>

namespace end_eff_shm
{
typedef struct
{
    double cur_positions_[END_EFF_DOF];
    double cur_velocities_[END_EFF_DOF];
    double cur_efforts_[END_EFF_DOF];

    double cmd_positions_[END_EFF_DOF];
    double cmd_velocities_[END_EFF_DOF];
    double cmd_efforts_[END_EFF_DOF];

    unsigned int control_modes_[END_EFF_DOF];
} EndEffShm;

}

#endif