#ifndef ROBOT_FUN_H_
#define ROBOT_FUN_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robot_info/robot_info.h>
#include <process_commu/robot_state_shm.h>
#include <process_commu/arm_shm.h>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#ifdef USE_END_EFFECTOR
#include <process_commu/end_eff_shm.h>
#endif

namespace robot_fun
{
class RobotFun : public rclcpp::Node
{
public:
    RobotFun(const std::string & node_name);
    ~RobotFun();

    void get_arm_joint_positions(std::vector<double> & positions);
    void get_arm_joint_velocities(std::vector<double> & velocities);
    void get_arm_joint_efforts(std::vector<double> & efforts);

    int set_arm_joint_positions(std::vector<double> & positions);
    int set_arm_joint_velocities(std::vector<double> & velocities);
    int set_arm_joint_efforts(std::vector<double> & efforts);

    std::string get_arm_control_mode(void);

    int arm_switch_controller(const std::string & start_controller);

#ifdef USE_END_EFFECTOR
    void get_end_eff_joint_positions(std::vector<double> & positions);
    void get_end_eff_joint_velocities(std::vector<double> & velocities);
    void get_end_eff_joint_efforts(std::vector<double> & efforts);

    int set_end_eff_joint_positions(std::vector<double> & positions);
    int set_end_eff_joint_velocities(std::vector<double> & velocities);
    int set_end_eff_joint_efforts(std::vector<double> & efforts);

    std::string get_end_eff_control_mode(void);
#endif

private:
    std::shared_ptr<rclcpp::Node> nh_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_state_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_positions_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_velocities_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_efforts_pub_;

    void callback_robot_state_sub_(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::shared_ptr<robot_info::RobotInfo> robot_ = 
        std::make_shared<robot_info::RobotInfo>();

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr 
        switch_controller_cli_;

    robot_state_shm::RobotStateShm *robot_state_shm_;
    int robot_state_shm_id_;
    int robot_state_sem_id_;

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