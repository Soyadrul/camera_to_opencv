from ament_index_python.resources import has_resource

from launch.actions import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode



from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # parameters
    camera_param_name = "camera"
    camera_param_default = str(0)
    camera_param = LaunchConfiguration(
        camera_param_name,
        default=camera_param_default,
    )
    camera_launch_arg = DeclareLaunchArgument(
        camera_param_name,
        default_value=camera_param_default,
        description="camera ID or name"
    )

    format_param_name = "format"
    format_param_default = str()
    format_param = LaunchConfiguration(
        format_param_name,
        default=format_param_default,
    )
    format_launch_arg = DeclareLaunchArgument(
        format_param_name,
        default_value=format_param_default,
        description="pixel format"
    )

    # camera node
    composable_nodes = [
        ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            parameters=[{
                "camera": camera_param,
                "width": 640,
                "height": 480,
                "format": format_param,
                "FrameDurationLimits": [500000,500000], # set fixed framerate of 2 Hz (500 ms)
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),

    ]

    # optionally add ImageViewNode to show camera image
    '''if has_resource("packages", "image_view"):
        composable_nodes += [
            ComposableNode(
                package='image_view',
                plugin='image_view::ImageViewNode',
                remappings=[('/image', '/camera/image_raw')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]'''

    # composable nodes in single container
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
    )
    
    return LaunchDescription([
        container,
        camera_launch_arg,
        format_launch_arg,
        Node(
            package='image_saver',
            executable='image_saver',
            name='image_saver_node',
            output='screen',
        ),
    ])
