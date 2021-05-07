#ifndef GAZEBO_CONTROL_PLUGIN_H_
#define GAZEBO_CONTROL_PLUGIN_H_

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Joint.hh>

#include <robot_info/robot_info.h>
#include <process_commu/arm_shm.h>
#include <process_commu/end_eff_shm.h>
#include <process_commu/robot_state_shm.h>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>

namespace gazebo_control_plugin
{
class ControlPlugin : public gazebo::ModelPlugin
{
public:
    ControlPlugin();
    ~ControlPlugin();

    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    void Update();

private:
    gazebo::physics::ModelPtr parent_model_;

    std::vector<gazebo::physics::JointPtr> arm_joints_;

    std::vector<gazebo::physics::JointPtr> end_eff_joints_;

    gazebo::event::ConnectionPtr update_connection_;

    std::shared_ptr<robot_info::RobotInfo> robot_ = std::make_shared<robot_info::RobotInfo>();

    arm_shm::ArmShm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;

    end_eff_shm::EndEffShm *end_eff_shm_;
    int end_eff_shm_id_;
    int end_eff_sem_id_;

    robot_state_shm::RobotStateShm *robot_state_shm_;
    int robot_state_shm_id_;
    int robot_state_sem_id_;
}; 

}

#endif
