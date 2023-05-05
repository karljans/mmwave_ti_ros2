import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # cfg_file = "6843ISK_3d.cfg"
    # cfg_file = "1843_3d.cfg"
    cfg_file = "xwr18xx_profile_2023_03_28T13_07_50_520.cfg"
    # cfg_file = "6843ISK_3d_editted.cfg"

    pkg_dir_path = get_package_share_directory('ti_mmwave_ros2_pkg')
    cfg_file_path = os.path.join(pkg_dir_path, 'cfg', cfg_file)

    mmwave_quick_config = Node(
        package='ti_mmwave_ros2_pkg',
        executable='mmWaveQuickConfig',
        name='mmwave_quick_config',
        output='screen',
        arguments=[cfg_file_path],
        parameters=[{
            "mmWaveCLI_name": "/mmWaveCLI",
        }],
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # ComposableNode(
                #     package='ti_mmwave_ros2_pkg',
                #     node_plugin='ti_mmwave_ros2_pkg::mmWaveCommSrv',
                #     # node_name='mmWaveCommSrv',
                #     parameters=[{
                #         "command_port": "/dev/ttyACM0",
                #         "command_rate": 115200,
                #         "mmWaveCLI_name": "/mmWaveCLI",
                #     }]
                # ),
                ComposableNode(
                    package='ti_mmwave_ros2_pkg',
                    plugin='ti_mmwave_ros2_pkg::mmWaveDataHdl',
                    name='mmWaveDataHdl',
                    parameters=[{
                        "data_port": "/dev/ttyACM1",
                        "data_rate": 921600,
                        "frame_id": "/ti_mmwave_0",
                        "max_allowed_elevation_angle_deg": 90,
                        "max_allowed_azimuth_angle_deg": 90,
                    }]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([
        mmwave_quick_config,
        # container,
    ])
