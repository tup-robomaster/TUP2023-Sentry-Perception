import yaml
from argparse import Namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os 

def generate_launch_description():
    ld = LaunchDescription()

    share_path_usb = get_package_share_directory('usb_cam')

    cam_config = os.path.join(share_path_usb, 'config', 'params.yaml')

    params_path_l = os.path.join(share_path_usb, 'config', 'usb_left','params.yaml')
    params_path_r = os.path.join(share_path_usb, 'config', 'usb_right','params.yaml')
    
    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output="log",
        name="usb_cam_node_l",
        parameters=[params_path_l],
        remappings=[("image_raw","/usb_left/image_raw"),
                    ("camera_info","/usb_left/camera_info")],
        respawn=True
        ))
    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output="log",
        name="usb_cam_node_r",
        parameters=[params_path_r],
        remappings=[("image_raw","/usb_right/image_raw"),
                    ("camera_info","/usb_right/camera_info")],
        respawn=True
        ))
    ld.add_action(Node(
        package = "perception_detector",
        name = 'perception_detector',
        executable = 'perception_detector_node',
        output="log",
        respawn=True
    ))
    # ld.add_action(ComposableNodeContainer(
    #         name='perception_detector_container',
    #         namespace='',
    #         output="log",
    #         package='rclcpp_components',
    #         executable='component_container',
    #         composable_node_descriptions=[
    #             ComposableNode(
    #                 package = "perception_detector",
    #                 name = 'perception_detector',
    #                 plugin = 'perception_detector::DetectorNode',
    #                 extra_arguments=[{
    #                     'use_intra_process_comms':True
    #                 }]
    #             ),
    #             ComposableNode(
    #                 package='usb_cam',
    #                 plugin='usb_cam::UsbCamNode',
    #                 name="usb_cam_node_l",
    #                 parameters=[params_path_l],
    #                 remappings=[("image_raw","/usb_left/image_raw"),
    #                             ("camera_info","/usb_left/camera_info")],
    #                 extra_arguments=[{
    #                     'use_intra_process_comms':True
    #                 }]
    #             ),
    #             ComposableNode(
    #                 package='usb_cam',
    #                 plugin='usb_cam::UsbCamNode',
    #                 name="usb_cam_node_r",
    #                 parameters=[params_path_r],
    #                 remappings=[("image_raw","/usb_right/image_raw"),
    #                             ("camera_info","/usb_right/camera_info")],
    #                 extra_arguments=[{
    #                     'use_intra_process_comms':True
    #                 }]
    #             ),
    #         ],
    #     ))
    # ld.add_action(show_image)
    # armor_detector_node)

    return ld





