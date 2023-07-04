import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file')
    tf_prefix = LaunchConfiguration('tf_prefix')
    base_frame = LaunchConfiguration('base_frame')
    parent_frame = LaunchConfiguration('parent_frame')
    cam_pos_x    = LaunchConfiguration('cam_x')
    cam_pos_y    = LaunchConfiguration('cam_y')
    cam_pos_z    = LaunchConfiguration('cam_z')
    cam_roll     = LaunchConfiguration('cam_roll')
    cam_pitch    = LaunchConfiguration('cam_pitch')
    cam_yaw      = LaunchConfiguration('cam_yaw')

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    tf_prefix_str = tf_prefix.perform(context)
    return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(depthai_prefix, "launch", "camera_urdf_launch.py"),
                    ),
                launch_arguments={'tf_prefix': tf_prefix,
                                  'base_frame': base_frame,
                                  'parent_frame': parent_frame,
                                  'cam_x': cam_pos_x,
                                  'cam_y': cam_pos_y,
                                  'cam_z': cam_pos_z,
                                  'cam_roll': cam_roll,
                                  'cam_pitch': cam_pitch,
                                  'cam_yaw': cam_yaw,
                                 }.items(),
            ),
            
            ComposableNodeContainer(
                name= "container",
                namespace=tf_prefix_str,
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    # Driver itself
                    ComposableNode(
                        namespace= tf_prefix_str,
                        package="depth_image_proc",
                        plugin="depth_image_proc::ConvertMetricNode",
                        name= "convert_metric_node",
                        remappings=[
                            ("image_raw", f"/{tf_prefix_str}/depth/image_raw"),
                            ("camera_info", f"/{tf_prefix_str}/depth/camera_info"),
                            ("image", f"/{tf_prefix_str}/depth/converted_depth"),
                        ],
                    ),

                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::RGBDCamera",
                        name=tf_prefix_str,
                        parameters=[params_file],
                    ),
                ],
                output="screen",
            )
        ]

def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value = '/var/novelte/config/camera_param.yaml',
                description='Full path to the camera_param parameters file to use for all launched nodes'),

            DeclareLaunchArgument(
                'tf_prefix',
                default_value = 'front_camera',
                description='prefix camera frame'),
            
            DeclareLaunchArgument(
                'base_frame',
                default_value = 'front_camera_link',
                description='camera link frame id'),
            
            DeclareLaunchArgument(
                'parent_frame',
                default_value = 'base_link',
                description='base link frame id'),
            
            DeclareLaunchArgument(
                'cam_x',
                default_value='0.0',
                description='Position X of the camera with respect to the base frame.'),

            DeclareLaunchArgument(
                'cam_y',
                default_value='0.0',
                description='Position Y of the camera with respect to the base frame.'),

            DeclareLaunchArgument(
                'cam_z',
                default_value='0.0',
                description='Position Z of the camera with respect to the base frame.'),

            DeclareLaunchArgument(
                'cam_roll',
                default_value='0.0',
                description='Roll orientation of the camera with respect to the base frame.'),

            DeclareLaunchArgument(
                'cam_pitch',
                default_value='0.0',
                description='Pitch orientation of the camera with respect to the base frame.'),

            DeclareLaunchArgument(
                'cam_yaw',
                default_value='0.0',
                description='Yaw orientation of the camera with respect to the base frame.'),
            OpaqueFunction(function=launch_setup),
        ]
    )