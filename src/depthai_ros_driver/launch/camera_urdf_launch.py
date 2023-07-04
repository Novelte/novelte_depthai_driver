from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('depthai_ros_driver')
    xacro_path = os.path.join(bringup_dir, 'urdf', 'depthai_descr.urdf.xacro')
    camera_model = LaunchConfiguration('camera_model')
    tf_prefix    = LaunchConfiguration('tf_prefix')
    base_frame   = LaunchConfiguration('base_frame')
    parent_frame = LaunchConfiguration('parent_frame')
    cam_pos_x    = LaunchConfiguration('cam_x')
    cam_pos_y    = LaunchConfiguration('cam_y')
    cam_pos_z    = LaunchConfiguration('cam_z')
    cam_roll     = LaunchConfiguration('cam_roll')
    cam_pitch    = LaunchConfiguration('cam_pitch')
    cam_yaw      = LaunchConfiguration('cam_yaw')
    tf_prefix_str = tf_prefix.perform(context)
    tf_prefix_node_str = tf_prefix_str + '_'
    return [

     Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{tf_prefix_node_str}oak_state_publisher',
            parameters=[{'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', tf_prefix, ' ',
                    'camera_model:=', camera_model, ' ',
                    'base_frame:=', base_frame, ' ',
                    'parent_frame:=', parent_frame, ' ',
                    'cam_pos_x:=', cam_pos_x, ' ',
                    'cam_pos_y:=', cam_pos_y, ' ',
                    'cam_pos_z:=', cam_pos_z, ' ',
                    'cam_roll:=', cam_roll, ' ',
                    'cam_pitch:=', cam_pitch, ' ',
                    'cam_yaw:=', cam_yaw
                ])}]
        ),       
    ]

def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_model',
                default_value='OAK-D',
                description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.'),

            DeclareLaunchArgument(
                'tf_prefix',
                default_value='oak',
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),

            DeclareLaunchArgument(
                'base_frame',
                default_value="oak-d_frame",
                description='Name of the base link.'),

            DeclareLaunchArgument(
                'parent_frame',
                default_value='oak-d-base-frame',
                description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.'),

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







