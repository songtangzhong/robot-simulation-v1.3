#ifndef ARM_INFO_H_
#define ARM_INFO_H_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

namespace arm_info
{
class ArmInfo
{
public:
    ArmInfo();
    ~ArmInfo();

    const unsigned int position_mode_ = (1<<0);
    const unsigned int velocity_mode_ = (1<<1);
    const unsigned int effort_mode_ = (1<<2);

    unsigned int dof_;

    std::vector<std::string> joint_names_;

    std::vector<double> cur_positions_;
    std::vector<double> cur_velocities_;
    std::vector<double> cur_efforts_;

    std::vector<double> cmd_positions_;
    std::vector<double> cmd_velocities_;
    std::vector<double> cmd_efforts_;

    std::vector<unsigned int> control_modes_;

    key_t shm_key_;
    key_t sem_key_;
};

}

#endif