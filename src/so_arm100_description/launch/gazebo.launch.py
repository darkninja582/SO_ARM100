import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('so_arm100_description')
    urdf_path = os.path.join(pkg, 'urdf', 'phosphobot_so_100.urdf')
    world_path = os.path.join(pkg, 'worlds', 'tabletop.world')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world_path,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }]
        ),

        TimerAction(period=5.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'so_arm100',
                    '-topic', 'robot_description',
                    '-x', '0.1',
                    '-y', '0.0',
                    '-z', '0.41',
                    '-Y', '1.5708',
                ],
                output='screen'
            )
        ]),

        TimerAction(period=8.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]),

        TimerAction(period=9.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller'],
                output='screen'
            )
        ]),

        TimerAction(period=10.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller'],
                output='screen'
            )
        ]),
    ])
