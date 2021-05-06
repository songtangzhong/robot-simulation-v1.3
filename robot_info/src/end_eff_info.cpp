#include <robot_info/end_eff_info.h>
#include <robot_info/robot_macro.h>

namespace end_eff_info
{
EndEffInfo::EndEffInfo()
{
    dof_ = END_EFF_DOF;

    joint_names_.resize(dof_);
    
    cur_positions_.resize(dof_);
    cur_velocities_.resize(dof_);
    cur_efforts_.resize(dof_);

    cmd_positions_.resize(dof_);
    cmd_velocities_.resize(dof_);
    cmd_efforts_.resize(dof_);

    control_modes_.resize(dof_);

    joint_names_ = {"panda_finger_joint1","panda_finger_joint2"};

    for (unsigned int j=0; j<dof_; j++)
    {
        cur_positions_[j] = cmd_positions_[j] = 0;
        cur_velocities_[j] = cmd_velocities_[j] = 0;
        cur_efforts_[j] = cmd_efforts_[j] = 0;

        control_modes_[j] = position_mode_; // default: position mode
    }

    shm_key_ = END_EFF_SHM_KEY;
    sem_key_ = END_EFF_SEM_KEY;
}

EndEffInfo::~EndEffInfo(){}

}
