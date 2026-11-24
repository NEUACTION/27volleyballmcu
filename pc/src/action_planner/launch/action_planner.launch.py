"""启动在线规划器与既有MCU串口节点。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    category = LaunchConfiguration('category')
    submode = LaunchConfiguration('submode')
    run_successed = LaunchConfiguration('run_successed')
    allow_pvt = LaunchConfiguration('allow_unvalidated_pvt')
    device = LaunchConfiguration('mcu_serial')
    baudrate = LaunchConfiguration('baudrate')
    return LaunchDescription([
        DeclareLaunchArgument('category', default_value='idle'),
        DeclareLaunchArgument('submode', default_value=''),
        DeclareLaunchArgument('run_successed', default_value='false'),
        DeclareLaunchArgument('allow_unvalidated_pvt', default_value='false'),
        DeclareLaunchArgument('mcu_serial', default_value='/dev/mcu'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        Node(package='action_planner', executable='action_planner_node',
             name='actionPlannerNode', output='screen', parameters=[{
                 'category': ParameterValue(category, value_type=str),
                 'submode': ParameterValue(submode, value_type=str),
                 'run_successed': ParameterValue(run_successed, value_type=bool),
                 'allow_unvalidated_pvt': ParameterValue(allow_pvt, value_type=bool),
             }]),
        Node(package='mcu', executable='mcu', name='mcuNode', output='screen', parameters=[{
            'mcu_serial': ParameterValue(device, value_type=str),
            'baudrate': ParameterValue(baudrate, value_type=int),
        }]),
    ])
