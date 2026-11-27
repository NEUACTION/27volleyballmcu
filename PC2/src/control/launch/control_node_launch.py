import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch

print(f"                              @@@              @@@    @@@                       ")
print(f"                             @@@@              @@@    @@@                       ")
print(f"                            @@@@@              @@@    @@@                       ")
print(f"                           @@@@@@              @@@                              ")
print(f"                          @@@@@@@  @@@@@@@@ @@@@@@@@@ @@@  @@@@@@@   @@@@@@@    ")
print(f"                         @@@@@@@@ @@@@@@@@@ @@@@@@@@@ @@@ @@@@@@@@@ @@@@@@@@@   ")
print(f"                        @@@@@@@@@ @@@@@@@@@ @@@@@@@@@ @@@ @@@@@@@@@ @@@@@@@@@   ")
print(f"                       @@@@@@ @@@ @@@@@@@@@ @@@@@@@@@ @@@ @@@@@@@@@ @@@@@@@@@   ")
print(f"                      @@@@@@  @@@ @@@          @@@    @@@ @@@   @@@ @@@   @@@   ")
print(f"                     @@@@@@   @@@ @@@          @@@    @@@ @@@   @@@ @@@   @@@   ")
print(f"                    @@@@@@    @@@ @@@          @@@    @@@ @@@   @@@ @@@   @@@   ")
print(f"                   @@@@@@     @@@ @@@          @@@    @@@ @@@   @@@ @@@   @@@   ")
print(f"                  @@@@@@      @@@ @@@@@@@@@    @@@    @@@ @@@@@@@@@ @@@   @@@   ")
print(f"                 @@@@@@       @@@ @@@@@@@@@    @@@    @@@ @@@@@@@@@ @@@   @@@   ")
print(f"                @@@@@@        @@@ @@@@@@@@@    @@@    @@@ @@@@@@@@@ @@@   @@@   ")
print(f"               @@@@@@         @@@  @@@@@@@@    @@@    @@@  @@@@@@@  @@@   @@@   ")

def generate_launch_description():

    declared_mode = DeclareLaunchArgument('mode', default_value='run')

    control_node = Node(
        package='control',
        executable='control',
        name='control_node',
        output='screen',
        parameters=[{'mode': LaunchConfiguration('mode')}]
    )

    return LaunchDescription([
        declared_mode,
        control_node
    ])
