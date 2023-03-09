from argparse import Namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os 

def generate_launch_description():
    ld = LaunchDescription()
    share_path_usb = get_package_share_directory('usb_cam')

    cam_config = os.path.join(share_path_usb, 'config', 'params.yaml')
    cam_config2 = os.path.join(share_path_usb, 'config', 'params_1.yaml')

    cam_node = Node(
        name = "usb_cam",
        package = "usb_cam",
        executable = "usb_cam_node_exe",
        parameters = [cam_config],
        output = "screen"
    )
    show_image = Node(
        package='usb_cam', executable='show_image.py', output='screen',
        # namespace=ns,
        # arguments=[image_manip_dir + "/data/mosaic.jpg"])
        remappings=[('image_raw', 'image_raw')]
        )
    cam_node2 = Node(
        name = "usb_cam2",
        package = "usb_cam",
        executable = "usb_cam_node_exe",
        parameters = [cam_config2],
        output = "screen"
        # remappings=[('image_raw', 'image_raw2')]
    )
    show_image2 = Node(
        package='usb_cam', executable='show_image.py', output='screen',
        # namespace=ns,
        # arguments=[image_manip_dir + "/data/mosaic.jpg"])
        remappings=[('image_raw', 'image_raw2')]
        )
    ld.add_action(cam_node)
    ld.add_action(cam_node2)
    ld.add_action(show_image)
    ld.add_action(show_image2)

    return ld





