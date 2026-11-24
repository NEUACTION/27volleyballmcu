/* 文件作用：启动PC侧ROS 2到MCU串口桥接进程；本文件不计算运动轨迹。 */
#include "mcu/mcuNode.hpp"

/* 中文名：串口桥接入口（main）。作用：初始化ROS 2并阻塞运行MCU通信节点。
 * 参数：argc为命令行项数（个），argv为命令行字符串数组；返回：正常退出为0。
 * 副作用：建立ROS上下文、节点和串口通信，结束后关闭ROS上下文。 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcu::mcuNode>());
    rclcpp::shutdown();
    return 0;
}
