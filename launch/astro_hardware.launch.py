from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events.process.process_exited import ProcessExited
from launch.launch_context import LaunchContext

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('astro_hardware'))

    astro_hardware_node = Node(
            package='astro_hardware',
            executable='astro_serial_node',
            namespace="astro",
            name="astro_serial",
            respawn=True,            
            respawn_delay=1
        )

    joy_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astro_hardware"),
                "launch/joy.launch.xml",
            )
        )
    )
    	
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astro_hardware"),
                "launch/lidar.launch.py",
            )
        )
    )
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astro_hardware"),
                "launch/realsense.launch.py",
            )
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time'),

        astro_hardware_node,
        joy_launch,
        lidar_launch,
        realsense_launch
    ])
