import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('vision_bringup'), 'launch'))

def generate_launch_description():
    
    from common import node_params, launch_params
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer
    from launch.actions import Shutdown
    from launch import LaunchDescription
    import yaml

    # load params for composable node
    with open(node_params, 'r') as f:
        camera_params = yaml.safe_load(f)['/camera_node']['ros__parameters']

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[camera_params],
    )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
            camera_node,
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
    )
 
    if (launch_params['camera'] == 'hik'):
        hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif (launch_params['camera'] == 'galaxy'):
        galaxy_camera_node = get_camera_node('galaxy_camera', 'galaxy_camera::GxCamera')
        cam_detector = get_camera_detector_container(galaxy_camera_node)

    return LaunchDescription([
        cam_detector,
    ])
