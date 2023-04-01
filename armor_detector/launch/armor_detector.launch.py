from argparse import Namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os 

def generate_launch_description():
    ld = LaunchDescription()

    share_path = get_package_share_directory('global_user')
    share_path_usb = get_package_share_directory('usb_cam')

    cam_config = os.path.join(share_path_usb, 'config', 'params.yaml')
    autoaim_config = os.path.join(share_path, 'config', 'autoaim.yaml')

    params_path_l = os.path.join(share_path_usb, 'config', 'usb_left','params.yaml')
    params_path_r = os.path.join(share_path_usb, 'config', 'usb_right','params.yaml')

    # ld.add_action(Node(
    #     package='usb_cam', executable='usb_cam_node_exe', output='screen',
    #     name="usb_cam_node_l",
    #     parameters=[params_path_l],
    #     remappings=[("image","/usb_left/image_raw"),
    #                 ("camera_info","/usb_left/camera_info")]
    #     ))
    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name="usb_cam_node_r",
        parameters=[params_path_r],
        remappings=[("image","/usb_right/image_raw"),
                    ("camera_info","/usb_right/camera_info")]
        ))
    ld.add_action(Node(
        package = "perception_detector",
        name = 'perception_detector',
        executable = 'perception_detector_node',
        output = 'screen'
    ))
    # ld.add_action(show_image)
    # armor_detector_node)

    return ld





