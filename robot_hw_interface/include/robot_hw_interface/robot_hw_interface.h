#ifndef ROBOT_HW_INTERFACE_H_
#define ROBOT_HW_INTERFACE_H_

#include <memory>
#include <string>
#include <vector>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <rclcpp/macros.hpp>

#include <rclcpp/rclcpp.hpp>
#include <robot_info/robot_info.h>
#include <process_commu/arm_shm.h>
#ifdef USE_END_EFFECTOR
#include <process_commu/end_eff_shm.h>
#endif
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>

namespace robot_hw
{
class RobotHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  double start_duration_sec_;
  double stop_duration_sec_;

  std::shared_ptr<robot_info::RobotInfo> robot_ = 
        std::make_shared<robot_info::RobotInfo>();

    arm_shm::ArmShm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;

#ifdef USE_END_EFFECTOR
    end_eff_shm::EndEffShm *end_eff_shm_;
    int end_eff_shm_id_;
    int end_eff_sem_id_;
#endif
};

}

#endif
