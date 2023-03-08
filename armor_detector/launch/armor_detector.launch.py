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

    cam_node = Node(
        name = "usb_cam",
        package = "usb_cam",
        executable = "usb_cam_node_exe",
        parameters = [cam_config],
        output = "screen"
    )
    # show_image = Node(
    #     package='usb_cam', executable='show_image.py', output='screen',
    #     # namespace=ns,
    #     # arguments=[image_manip_dir + "/data/mosaic.jpg"])
    #     # remappings=[('image_in', 'image_raw')]
    #     )

    armor_detector_node = Node(
        name = 'armor_detector',
        package = "armor_detector",
        executable = 'armor_detector_node',
        parameters = [autoaim_config],
        output = 'screen'
    )

    ld.add_action(cam_node)
    # ld.add_action(show_image)
    ld.add_action(armor_detector_node)

    return ld





