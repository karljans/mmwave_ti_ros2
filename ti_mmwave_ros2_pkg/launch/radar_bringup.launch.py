import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler

from launch.event_handlers import OnProcessExit
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ns = "can"
    
    cfg_file = "xwr18xx_profile_2023_04_06T14_24_07_323.cfg"
    pkg_dir_path = get_package_share_directory('ti_mmwave_ros2_pkg')
    cfg_file_path = os.path.join(pkg_dir_path, 'cfg', cfg_file)

    mmwave_quick_config = Node(
        package='ti_mmwave_ros2_pkg',
        executable='mmWaveQuickConfig',
        name='mmwave_quick_config',
        output='screen',
        namespace= ns,
        arguments=[cfg_file_path],
        parameters=[{
            "mmWaveCLI_name": "/"+ns+"/mmWaveCLI",
            "namespace": ns,
        }],
    )

    mmwave_comm_srv_node = Node(
        package='ti_mmwave_ros2_pkg',
        executable='mmwave_comm_srv_node',
        name='mmWaveCommSrvNode',
        namespace= ns,
        output='screen',
        parameters=[{
            "command_port": "/dev/ttyACM0",
            "command_rate": 115200,
            "mmWaveCLI_name": "/"+ns+"/mmWaveCLI",
        }],
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='radar_container',
            namespace= ns,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ti_mmwave_ros2_pkg',
                    plugin='ti_mmwave_ros2_pkg::mmWaveDataHdl',
                    name='mmWaveDataHdl',
                    namespace= ns,
                    parameters=[{
                        "data_port": "/dev/ttyACM1",
                        "data_rate": 921600,
                        "frame_id": "ti_mmwave_0",
                        "max_allowed_elevation_angle_deg": 90,
                        "max_allowed_azimuth_angle_deg": 90,
                    }]
                ),

            ],
            output='screen',
    )

    return launch.LaunchDescription([
        mmwave_comm_srv_node,
        mmwave_quick_config,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=mmwave_quick_config,
                on_exit=[container],
            )
        ),
    ])
