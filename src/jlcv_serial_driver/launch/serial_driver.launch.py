import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown 


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('jlcv_serial_driver'), 'config', 'serial_driver.yaml')

    jlcv_serial_driver_node = Node(
        package='jlcv_serial_driver',
        executable='jlcv_serial_driver_node',
        namespace='',
        output='both',
        emulate_tty=True,
        parameters=[config],
        on_exit=Shutdown(),
    )

    return LaunchDescription([jlcv_serial_driver_node])
