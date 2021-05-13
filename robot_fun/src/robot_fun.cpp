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

    cmd_positions_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controllers/commands", 100);
    cmd_velocities_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controllers/commands", 100);
    cmd_efforts_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/effort_controllers/commands", 100);
    
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

    ///////////////////////////////////////////////////////////////////////////////////////////
    arm_shm_id_ = shm_common::create_shm(robot_->arm_->shm_key_, &arm_shm_);
    if (arm_shm_id_ == SHM_STATE_NO)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotHardware"), 
            "Create arm shared memory failed.");
    }

    arm_sem_id_ = sem_common::create_semaphore(robot_->arm_->sem_key_);
    if (arm_sem_id_ == SEM_STATE_NO)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotHardware"), 
            "Create arm semaphore failed.");
    }

#ifdef USE_END_EFFECTOR
  ///////////////////////////////////////////////////////////////////////////////////////////
  end_eff_shm_id_ = shm_common::create_shm(robot_->end_eff_->shm_key_, &end_eff_shm_);
  if (end_eff_shm_id_ == SHM_STATE_NO)
  {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardware"), 
          "Create end-effector shared memory failed.");
  }

  end_eff_sem_id_ = sem_common::create_semaphore(robot_->end_eff_->sem_key_);
  if (end_eff_sem_id_ == SEM_STATE_NO)
  {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardware"), 
          "Create end-effector semaphore failed.");
  }
#endif
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

void RobotFun::get_arm_joint_positions(std::vector<double> & positions)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        positions[j] = robot_state_shm_->cur_arm_positions_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

void RobotFun::get_arm_joint_velocities(std::vector<double> & velocities)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        velocities[j] = robot_state_shm_->cur_arm_velocities_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

void RobotFun::get_arm_joint_efforts(std::vector<double> & efforts)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        efforts[j] = robot_state_shm_->cur_arm_efforts_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

int RobotFun::set_arm_joint_positions(std::vector<double> & positions)
{
    if (positions.size() != robot_->arm_->dof_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), 
        "Failed to set arm joint positions, size error.");

        return -1;
    }

    auto cmd = std_msgs::msg::Float64MultiArray();
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        cmd.data.push_back(positions[j]);
    }
    cmd_positions_pub_->publish(cmd);

    return 0;
}

int RobotFun::set_arm_joint_velocities(std::vector<double> & velocities)
{
    if (velocities.size() != robot_->arm_->dof_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), 
        "Failed to set arm joint velocities, size error.");

        return -1;
    }

    auto cmd = std_msgs::msg::Float64MultiArray();
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        cmd.data.push_back(velocities[j]);
    }
    cmd_velocities_pub_->publish(cmd);

    return 0;
}

int RobotFun::set_arm_joint_efforts(std::vector<double> & efforts)
{
    if (efforts.size() != robot_->arm_->dof_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), 
        "Failed to set arm joint efforts, size error.");

        return -1;
    }

    auto cmd = std_msgs::msg::Float64MultiArray();
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        cmd.data.push_back(efforts[j]);
    }
    cmd_efforts_pub_->publish(cmd);

    return 0;
}

std::string RobotFun::get_arm_control_mode(void)
{
    std::string control_mode;

    sem_common::semaphore_p(arm_sem_id_);
    if (arm_shm_->control_modes_[0] & robot_->arm_->position_mode_)
    {
        control_mode = "position_mode";
    }
    else if (arm_shm_->control_modes_[0] & robot_->arm_->velocity_mode_)
    {
        control_mode = "velocity_mode";
    }
    else if (arm_shm_->control_modes_[0] & robot_->arm_->effort_mode_)
    {
        control_mode = "effort_mode";
    }
    sem_common::semaphore_v(arm_sem_id_);

    return control_mode;
}

#ifdef USE_END_EFFECTOR
void RobotFun::get_end_eff_joint_positions(std::vector<double> & positions)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->end_eff_->dof_; j++)
    {
        positions[j] = robot_state_shm_->cur_end_eff_positions_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

void RobotFun::get_end_eff_joint_velocities(std::vector<double> & velocities)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->end_eff_->dof_; j++)
    {
        velocities[j] = robot_state_shm_->cur_end_eff_velocities_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

void RobotFun::get_end_eff_joint_efforts(std::vector<double> & efforts)
{
    sem_common::semaphore_p(robot_state_sem_id_);
    for (unsigned int j=0; j< robot_->end_eff_->dof_; j++)
    {
        efforts[j] = robot_state_shm_->cur_end_eff_efforts_[j];
    }
    sem_common::semaphore_v(robot_state_sem_id_);
}

int RobotFun::set_end_eff_joint_positions(std::vector<double> & positions)
{
    if (positions.size() != robot_->end_eff_->dof_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), 
        "Failed to set end-effector joint positions, size error.");

        return -1;
    }

    sem_common::semaphore_p(end_eff_sem_id_);
    for (unsigned int j=0; j< robot_->end_eff_->dof_; j++)
    {
        end_eff_shm_->control_modes_[j] = robot_->end_eff_->position_mode_;
        end_eff_shm_->cmd_positions_[j] = positions[j];
    }
    sem_common::semaphore_v(end_eff_sem_id_);

    return 0;
}

int RobotFun::set_end_eff_joint_velocities(std::vector<double> & velocities)
{
    if (velocities.size() != robot_->end_eff_->dof_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), 
        "Failed to set end-effector joint velocities, size error.");

        return -1;
    }

    sem_common::semaphore_p(end_eff_sem_id_);
    for (unsigned int j=0; j< robot_->end_eff_->dof_; j++)
    {
        end_eff_shm_->control_modes_[j] = robot_->end_eff_->velocity_mode_;
        end_eff_shm_->cmd_velocities_[j] = velocities[j];
    }
    sem_common::semaphore_v(end_eff_sem_id_);

    return 0;
}

int RobotFun::set_end_eff_joint_efforts(std::vector<double> & efforts)
{
    if (efforts.size() != robot_->end_eff_->dof_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), 
        "Failed to set end-effector joint efforts, size error.");

        return -1;
    }

    sem_common::semaphore_p(end_eff_sem_id_);
    for (unsigned int j=0; j< robot_->end_eff_->dof_; j++)
    {
        end_eff_shm_->control_modes_[j] = robot_->end_eff_->effort_mode_;
        end_eff_shm_->cmd_efforts_[j] = efforts[j];
    }
    sem_common::semaphore_v(end_eff_sem_id_);

    return 0;
}

std::string RobotFun::get_end_eff_control_mode(void)
{
    std::string control_mode;

    sem_common::semaphore_p(end_eff_sem_id_);
    if (end_eff_shm_->control_modes_[0] & robot_->end_eff_->position_mode_)
    {
        control_mode = "position_mode";
    }
    else if (end_eff_shm_->control_modes_[0] & robot_->end_eff_->velocity_mode_)
    {
        control_mode = "velocity_mode";
    }
    else if (end_eff_shm_->control_modes_[0] & robot_->end_eff_->effort_mode_)
    {
        control_mode = "effort_mode";
    }
    sem_common::semaphore_v(end_eff_sem_id_);

    return control_mode;
}
#endif

}
