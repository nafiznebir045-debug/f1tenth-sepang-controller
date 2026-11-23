import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_gym_ros')
    config_file = os.path.join(pkg_share, 'config', 'sim.yaml')
    rviz_config = os.path.join(pkg_share, 'launch', 'gym_bridge.rviz')
    xacro_file = os.path.join(pkg_share, 'launch', 'ego_racecar.xacro')

    try:
        robot_desc = xacro.process_file(xacro_file).toxml()
    except Exception as e:
        urdf_file = os.path.join(pkg_share, 'launch', 'ego_racecar.urdf')
        if os.path.exists(urdf_file):
            with open(urdf_file, 'r') as f:
                robot_desc = f.read()
        else:
            robot_desc = ''

    gym_bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config_file]
    )

    ego_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }],
        remappings=[
            ('/robot_description', '/ego_robot_description')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        gym_bridge_node,
        ego_robot_state_publisher,
        rviz_node
    ])
