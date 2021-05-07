#include <robot_fun/robot_fun.h>
#include <memory>

using std::placeholders::_1;

namespace robot_fun
{
RobotFun::RobotFun(const std::string & node_name)
: rclcpp::Node(node_name)
{
    robot_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 100, std::bind(&RobotFun::callback_robot_state_sub_, this, _1));

    robot_state_shm_id_ = shm_common::create_shm(robot_->state_shm_key_, &robot_state_shm_);
    if (robot_state_shm_id_ == SHM_STATE_NO)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), 
            "Create robot state shared memory failed.");
    }

    robot_state_sem_id_ = sem_common::create_semaphore(robot_->state_sem_key_);
    if (robot_state_sem_id_ == SEM_STATE_NO)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), 
            "Create robot state semaphore failed.");
    }
}

RobotFun::~RobotFun(){}

void RobotFun::callback_robot_state_sub_(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    unsigned int k = 0;
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< msg->name.size(); j++)
    {
        for (unsigned int i=0; i< robot_->arm_->dof_; i++)
        {
            if (msg->name[j] == robot_->arm_->joint_names_[i])
            {
                robot_state_shm_->cur_arm_positions_[i] = msg->position[j];
                robot_state_shm_->cur_arm_velocities_[i] = msg->velocity[j];
                robot_state_shm_->cur_arm_efforts_[i] = msg->effort[j];

                goto next_loop;
            }
        }

#ifdef USE_END_EFFECTOR
        for (unsigned int i=0; i< robot_->end_eff_->dof_; i++)
        {
            if (msg->name[j] == robot_->end_eff_->joint_names_[i])
            {
                robot_state_shm_->cur_end_eff_positions_[i] = msg->position[j];
                robot_state_shm_->cur_end_eff_velocities_[i] = msg->velocity[j];
                robot_state_shm_->cur_end_eff_efforts_[i] = msg->effort[j];

                goto next_loop;
            }
        }
#endif

        next_loop:
            k++;
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

void RobotFun::get_arm_joint_positions(double * positions)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        *(positions+j) = robot_state_shm_->cur_arm_positions_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

void RobotFun::get_arm_joint_velocities(double * velocities)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        *(velocities+j) = robot_state_shm_->cur_arm_velocities_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

void RobotFun::get_arm_joint_efforts(double * efforts)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        *(efforts+j) = robot_state_shm_->cur_arm_efforts_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

#ifdef USE_END_EFFECTOR
void RobotFun::get_end_eff_joint_positions(double * positions)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->end_eff_->dof_; j++)
    {
        *(positions+j) = robot_state_shm_->cur_end_eff_positions_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

void RobotFun::get_end_eff_joint_velocities(double * velocities)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->end_eff_->dof_; j++)
    {
        *(velocities+j) = robot_state_shm_->cur_end_eff_velocities_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

void RobotFun::get_end_eff_joint_efforts(double * efforts)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->end_eff_->dof_; j++)
    {
        *(efforts+j) = robot_state_shm_->cur_end_eff_efforts_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}
#endif

}
