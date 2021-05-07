#include <chrono>
#include <memory>
#include <string>
#include <controller_manager_msgs/srv/load_start_controller.hpp>
#include <controller_manager_msgs/srv/load_configure_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/controller_configure.h>
#include <robot_info/robot_macro.h>

using namespace std::chrono_literals;

namespace controller_configure
{
ControllerConfigure::ControllerConfigure(const std::string & node_name)
{
    nh_ = std::make_shared<rclcpp::Node>(node_name);

    load_start_controller_cli_ = 
    nh_->create_client<controller_manager_msgs::srv::LoadStartController>("/controller_manager/load_and_start_controller");

    load_configure_controller_cli_ = 
    nh_->create_client<controller_manager_msgs::srv::LoadConfigureController>("/controller_manager/load_and_configure_controller");

    switch_controller_cli_ = 
    nh_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    arm_shm_id_ = shm_common::create_shm(robot_->arm_->shm_key_, &arm_shm_);
    if (arm_shm_id_ == SHM_STATE_NO)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ControllerConfigure"), 
            "Create arm shared memory failed.");
    }

    arm_sem_id_ = sem_common::create_semaphore(robot_->arm_->sem_key_);
    if (arm_sem_id_ == SEM_STATE_NO)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ControllerConfigure"), 
            "Create arm semaphore failed.");
    }

    cmd_positions_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controllers/commands", 100);
    cmd_velocities_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controllers/commands", 100);
    cmd_efforts_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands", 100);
}

ControllerConfigure::~ControllerConfigure(){}

void ControllerConfigure::load_start_controller(const std::string & controller_name)
{
    auto request = std::make_shared<controller_manager_msgs::srv::LoadStartController::Request>();
    request->name = controller_name;
    while (!load_start_controller_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("load_start_controller"), 
                "Interrupted while waiting for the service. Exiting.");
        }

        RCLCPP_INFO(rclcpp::get_logger("load_start_controller"), 
            "service [/controller_manager/load_and_start_controller] not available, waiting again...");
    }

    auto result = load_start_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("load_start_controller"), 
            "load and start %s successfully.", controller_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("load_start_controller"), 
            "Failed to load and start %s.", controller_name.c_str());
    }
}

void ControllerConfigure::load_configure_controller(const std::string & controller_name)
{
    auto request = std::make_shared<controller_manager_msgs::srv::LoadConfigureController::Request>();
    request->name = controller_name;
    while (!load_configure_controller_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("load_configure_controller"), 
                "Interrupted while waiting for the service. Exiting.");
        }

        RCLCPP_INFO(rclcpp::get_logger("load_configure_controller"), 
            "service [/controller_manager/load_and_configure_controller] not available, waiting again...");
    }

    auto result = load_configure_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("load_configure_controller"), 
            "load and configure %s successfully.", controller_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("load_configure_controller"), 
            "Failed to load and start %s.", controller_name.c_str());
    }
}

void ControllerConfigure::switch_controller(const std::string & start_controller, const std::string & stop_controller)
{
    std::vector<std::string> start_controller_ = {start_controller};
    std::vector<std::string> stop_controller_ = {stop_controller};
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers = start_controller_;
    request->stop_controllers = stop_controller_;
    request->strictness = request->BEST_EFFORT;
    request->start_asap = false;
    request->timeout = rclcpp::Duration(static_cast<rcl_duration_value_t>(0.0));
    while (!switch_controller_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("switch_controller"), 
                "Interrupted while waiting for the service. Exiting.");
        }

        RCLCPP_INFO(rclcpp::get_logger("switch_controller"), 
            "service [/controller_manager/switch_controller] not available, waiting again...");
    }

    auto result = switch_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        sem_common::semaphore_p(arm_sem_id_);
        if (start_controller == "position_controllers")
        {
            // Set current arm joint positions to commands by ros2 controller manager,
            // not by shared memory.
            double cur_arm_positions[ARM_DOF];
            robot_fun_->get_arm_joint_positions(cur_arm_positions);
            auto cmd = std_msgs::msg::Float64MultiArray();
            for (unsigned int j=0; j< robot_->arm_->dof_; j++)
            {
                cmd.data.push_back(cur_arm_positions[j]);
                arm_shm_->control_modes_[j] = robot_->arm_->position_mode_;
            }
            cmd_positions_pub_->publish(cmd);
        }
        else if (start_controller == "velocity_controllers")
        {
            // Set current arm joint velocities (zeros) to commands by ros2 controller manager,
            // not by shared memory.
            double cur_arm_velocities[ARM_DOF];
            auto cmd = std_msgs::msg::Float64MultiArray();
            for (unsigned int j=0; j< robot_->arm_->dof_; j++)
            {
                cur_arm_velocities[j] = 0;
                cmd.data.push_back(cur_arm_velocities[j]);
                arm_shm_->control_modes_[j] = robot_->arm_->velocity_mode_;
            }
            cmd_velocities_pub_->publish(cmd);
        }
        else if (start_controller == "effort_controllers")
        {
            // Set current arm joint efforts (zeros) to commands by ros2 controller manager,
            // not by shared memory.
            double cur_arm_efforts[ARM_DOF];
            auto cmd = std_msgs::msg::Float64MultiArray();
            for (unsigned int j=0; j< robot_->arm_->dof_; j++)
            {
                cur_arm_efforts[j] = 0;
                cmd.data.push_back(cur_arm_efforts[j]);
                arm_shm_->control_modes_[j] = robot_->arm_->effort_mode_;
            }
            cmd_efforts_pub_->publish(cmd);
        }
        sem_common::semaphore_v(arm_sem_id_);

        RCLCPP_INFO(rclcpp::get_logger("switch_controller"), 
            "switch %s to %s successfully.", stop_controller.c_str(), start_controller.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("switch_controller"), 
            "Failed to switch %s to %s.", stop_controller.c_str(), start_controller.c_str());
    }
}

}
