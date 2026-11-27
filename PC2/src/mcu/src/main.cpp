#include <chrono>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "mcu/mcuNode.hpp"

std::shared_ptr<mcuNode> mcu_node;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    mcu_node = std::make_shared<mcuNode>();
    
    rclcpp::spin(mcu_node);

    rclcpp::shutdown();
    return(0);
    
}
