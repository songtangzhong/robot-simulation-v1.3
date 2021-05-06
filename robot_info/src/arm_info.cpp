#include <robot_info/arm_info.h>
#include <robot_info/robot_macro.h>

namespace arm_info
{
ArmInfo::ArmInfo()
{
    dof_ = ARM_DOF;

    joint_names_.resize(dof_);
    
    cur_positions_.resize(dof_);
    cur_velocities_.resize(dof_);
    cur_efforts_.resize(dof_);

    cmd_positions_.resize(dof_);
    cmd_velocities_.resize(dof_);
    cmd_efforts_.resize(dof_);

    control_modes_.resize(dof_);

    joint_names_ = {"panda_joint1","panda_joint2","panda_joint3","panda_joint4",
                    "panda_joint5","panda_joint6","panda_joint7"};

    for (unsigned int j=0; j<dof_; j++)
    {
        cur_positions_[j] = cmd_positions_[j] = 0;
        cur_velocities_[j] = cmd_velocities_[j] = 0;
        cur_efforts_[j] = cmd_efforts_[j] = 0;

        control_modes_[j] = position_mode_; // default: position mode
    }
    cur_positions_[3] = cmd_positions_[3] =  -1.5;
    cur_positions_[3] = cmd_positions_[5] = 1.88;

    shm_key_ = ARM_SHM_KEY;
    sem_key_ = ARM_SEM_KEY;
}

ArmInfo::~ArmInfo(){}

}
