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

def controller_description():
    Node(
        package='astro_hardware',
        executable='astro_serial_node',
        namespace="astro_hardware",
        name="astro_serial"
    )


def on_exit_restart(event:ProcessExited, context:LaunchContext):
    print("\n\nProcess [{}] exited, pid: {}, return code: {}\n\n".format(
        event.action.name, event.pid, event.returncode))
    if event.returncode != 0 and 'controller' in event.action.name:
        return controller_description() # respawn node action

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('astro_hardware'))

    astro_hardware_node = Node(
            package='astro_hardware',
            executable='astro_serial_node',
            namespace="astro_hardware",
            name="astro_serial"
        ),

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

#     robot_localization_node = Node(
#        package='robot_localization',
#        executable='ekf_node',
#        name='ekf_filter_node',
#        output='screen',
#        parameters=[os.path.join(pkg_path, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
# )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time'),

        astro_hardware_node,
        joy_launch,
        # robot_localization_node,
        lidar_launch,
        realsense_launch,
        RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart))
    ])
