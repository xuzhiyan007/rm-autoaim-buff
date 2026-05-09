import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PythonExpression, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import Shutdown, DeclareLaunchArgument
from launch.conditions import IfCondition

use_serial = LaunchConfiguration('use_serial')
declare_use_serial_cmd = DeclareLaunchArgument(
    'use_serial',
    default_value=' True',
    description='Whether use serial port')

use_video = LaunchConfiguration('use_video')
declare_use_video_cmd = DeclareLaunchArgument(
    'use_video',
    default_value=' False',
    description='Whether use video')


launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('vision_bringup'), 'config', 'launch_params.yaml')))


robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('jlcv_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
    ' xyz:=', launch_params['world2camera']['xyz'],
    ' rpy:=', launch_params['world2camera']['rpy'],
    ' xyzw:=', launch_params['world2camera']['xyzw']])

node_params = os.path.join(
    get_package_share_directory('vision_bringup'), 'config', 'node_params.yaml')

joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[{'rate': 600}],
    condition=IfCondition(PythonExpression(["not ", use_serial]))
)

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                # "use_sim_time": True
                 'publish_frequency': 1000.0
                }],
    on_exit=Shutdown(),
)

armor_processor_node = Node(
    package='armor_processor',
    executable='armor_processor_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
)

solve_angle_node = Node(
    package='solve_angle',
    executable='solve_angle_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
)

serial_driver_node = Node(
    package='jlcv_serial_driver',
    executable='jlcv_serial_driver_node',
    name='jlcv_serial_driver',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
    condition=IfCondition(use_serial),
    on_exit=Shutdown(),
)