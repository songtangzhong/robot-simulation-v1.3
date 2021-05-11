#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/robot_hw_interface.h>

namespace robot_hw
{
hardware_interface::return_type RobotHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  start_duration_sec_ = stod(info_.hardware_parameters["start_duration_sec"]);
  stop_duration_sec_ = stod(info_.hardware_parameters["stop_duration_sec"]);

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

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_POSITION, &robot_->arm_->cur_positions_[j]));
  }
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_VELOCITY, &robot_->arm_->cur_velocities_[j]));
  }
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_EFFORT, &robot_->arm_->cur_efforts_[j]));
  }

#ifdef USE_END_EFFECTOR
  //////////////////////////////////////////////////////////////////////////////////////////////
  for (unsigned int j = 0; j < robot_->end_eff_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->end_eff_->joint_names_[j], hardware_interface::HW_IF_POSITION, &robot_->end_eff_->cur_positions_[j]));
  }
  for (unsigned int j = 0; j < robot_->end_eff_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->end_eff_->joint_names_[j], hardware_interface::HW_IF_VELOCITY, &robot_->end_eff_->cur_velocities_[j]));
  }
  for (unsigned int j = 0; j < robot_->end_eff_->dof_; j++) 
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        robot_->end_eff_->joint_names_[j], hardware_interface::HW_IF_EFFORT, &robot_->end_eff_->cur_efforts_[j]));
  }
#endif

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_POSITION, &robot_->arm_->cmd_positions_[j]));
  }
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_VELOCITY, &robot_->arm_->cmd_velocities_[j]));
  }
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        robot_->arm_->joint_names_[j], hardware_interface::HW_IF_EFFORT, &robot_->arm_->cmd_efforts_[j]));
  }

  return command_interfaces;
}


hardware_interface::return_type RobotHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "Starting ...please wait...");

  /*for (int i = 0; i <= start_duration_sec_; i++) 
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RobotHardware"),
      "%.1f seconds left...", start_duration_sec_ - i);
  }*/

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "System Sucessfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "Stopping ...please wait...");

  /*for (int i = 0; i <= stop_duration_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RobotHardware"),
      "%.1f seconds left...", stop_duration_sec_ - i);
  }*/

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("RobotHardware"),
    "System sucessfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::read()
{
  sem_common::semaphore_p(arm_sem_id_);
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    robot_->arm_->cur_positions_[j] = arm_shm_->cur_positions_[j];
    robot_->arm_->cur_velocities_[j] = arm_shm_->cur_velocities_[j];
    robot_->arm_->cur_efforts_[j] = arm_shm_->cur_efforts_[j];
  }

#ifdef USE_END_EFFECTOR
  sem_common::semaphore_p(end_eff_sem_id_);
  for (unsigned int j = 0; j < robot_->end_eff_->dof_; j++) 
  {
    robot_->end_eff_->cur_positions_[j] = end_eff_shm_->cur_positions_[j];
    robot_->end_eff_->cur_velocities_[j] = end_eff_shm_->cur_velocities_[j];
    robot_->end_eff_->cur_efforts_[j] = end_eff_shm_->cur_efforts_[j];
  }
  sem_common::semaphore_v(end_eff_sem_id_);
#endif

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::write()
{
  for (unsigned int j = 0; j < robot_->arm_->dof_; j++) 
  {
    if ((robot_->arm_->control_modes_[j]=arm_shm_->control_modes_[j]) & robot_->arm_->position_mode_) 
    {
      arm_shm_->cmd_positions_[j] = robot_->arm_->cmd_positions_[j];
    }
    else if ((robot_->arm_->control_modes_[j]=arm_shm_->control_modes_[j]) & robot_->arm_->velocity_mode_) 
    {
      arm_shm_->cmd_velocities_[j] = robot_->arm_->cmd_velocities_[j];
    }
    else if ((robot_->arm_->control_modes_[j]=arm_shm_->control_modes_[j]) & robot_->arm_->effort_mode_) 
    {
      arm_shm_->cmd_efforts_[j] = robot_->arm_->cmd_efforts_[j];
    }
  }
  sem_common::semaphore_v(arm_sem_id_);
  
  return hardware_interface::return_type::OK;
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  robot_hw::RobotHardware,
  hardware_interface::SystemInterface
)
