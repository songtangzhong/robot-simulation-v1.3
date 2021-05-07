#include <process_commu/sem_common.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    int sem_id = sem_common::create_semaphore(1234);
    if (sem_id != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), "Create semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), "Create semaphore failed.");
        return 0;
    }

    if (sem_common::semaphore_p(sem_id) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), "Get semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), "Get semaphore failed.");
        return 0;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    if (sem_common::semaphore_v(sem_id) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), "Release semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), "Release semaphore failed.");
        return 0;
    }

    if (sem_common::delete_semaphore(sem_id) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test"), "Delete semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test"), "Delete semaphore failed.");
        return 0;
    }
    
    rclcpp::shutdown();
    return 0;
}
