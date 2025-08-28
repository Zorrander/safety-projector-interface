from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

config_file = os.path.join(
    get_package_share_directory('tuni_whitegoods_transformations'),
    'config',
    'example_params.yaml'
)


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='transform_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='tuni_whitegoods_transformations',
                    plugin='tuni_whitegoods_transformations::CameraTransformNode',
                    name='camera_transform',
                    parameters=[config_file]
                ),
                ComposableNode(
                    package='tuni_whitegoods_transformations',
                    plugin='tuni_whitegoods_transformations::ProjectorTransformNode',
                    name='projector_transform',
                    parameters=[config_file]
                ),
                ComposableNode(
                    package='tuni_whitegoods_transformations',
                    plugin='tuni_whitegoods_transformations::TFTransformNode',
                    name='tf_transform'
                ),
            ],
            output='screen',
        )
    ])