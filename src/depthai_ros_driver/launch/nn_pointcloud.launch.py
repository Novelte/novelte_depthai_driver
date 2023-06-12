import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    rviz_config = os.path.join(depthai_prefix, "config", "rgbd.rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="False"),
            DeclareLaunchArgument("mxid", default_value=""),
            DeclareLaunchArgument("namespace", default_value=""),
            Node(
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(depthai_prefix, "launch", "description.launch.py")
                )
            ),
            ComposableNodeContainer(
                name="container",
                namespace=LaunchConfiguration("namespace"),
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        namespace=LaunchConfiguration("namespace"),
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::NnPointcloud",
                        name="camera",
                        parameters=[{
                            "i_camera_mxid": LaunchConfiguration("mxid"),
                            "i_rgb_fps": 30.0,
                            "i_mono_fps" : 30.0,
                            "i_mono_resolution" : "400",
                            "i_align_depth" : True,
                            "i_lr_check" : True,
                            "i_subpixel" : True,
                            # "i_extended_disp" : True,
                            "i_rectify_edge_fill_color" : 0,
                            "i_enable_speckle_filter" : True,
                            "i_speckle_range" : 10,
                            "i_enable_temporal_filter" : True,
                            "i_enable_spatial_filter" : True,
                            "i_hole_filling_radius" : 2,
                            "i_spatial_filter_iterations" : 1,
                            "i_threshold_filter_min_range" : 700,
                            "i_threshold_filter_max_range" : 4000,
                            "i_decimation_factor" : 1,
                        }],
                    ),
                ],
                output="screen",
            ),
        ]
    )
