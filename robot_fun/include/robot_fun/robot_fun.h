#ifndef ROBOT_FUN_H_
#define ROBOT_FUN_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robot_info/robot_info.h>
#include <process_commu/robot_state_shm.h>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace robot_fun
{
class RobotFun : public rclcpp::Node
{
public:
    RobotFun(const std::string & node_name);
    ~RobotFun();

    void get_arm_joint_positions(double * positions);
    void get_arm_joint_velocities(double * velocities);
    void get_arm_joint_efforts(double * efforts);

    int set_arm_joint_positions(std::vector<double> & positions);

#ifdef USE_END_EFFECTOR
    void get_end_eff_joint_positions(double * positions);
    void get_end_eff_joint_velocities(double * velocities);
    void get_end_eff_joint_efforts(double * efforts);
#endif

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_state_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_positions_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_velocities_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_efforts_pub_;

    void callback_robot_state_sub_(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::shared_ptr<robot_info::RobotInfo> robot_ = 
        std::make_shared<robot_info::RobotInfo>();

    robot_state_shm::RobotStateShm *robot_state_shm_;
    int robot_state_shm_id_;
    int robot_state_sem_id_;
};

}

#endif