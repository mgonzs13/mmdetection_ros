import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mmdetection_bringup_path = get_package_share_directory(
        "mmdetection_bringup")

    #
    # ARGS
    #
    network_config = LaunchConfiguration("network_config")
    network_config_cmd = DeclareLaunchArgument(
        "network_config",
        default_value=os.path.join(
            mmdetection_bringup_path,
            "config", "yolact",
            "yolact_r101_1x8_coco.py"),
        description="Network configuration file (.py)")

    weights = LaunchConfiguration("weights")
    weights_cmd = DeclareLaunchArgument(
        "weights",
        default_value=os.path.join(
            mmdetection_bringup_path,
            "config", "yolact",
            "yolact_r101_1x8_coco_20200908-4cbe9101.pth"),
        description="Weights file (.pth)")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Device to use (GPU/CPU)")

    enable = LaunchConfiguration("enable")
    enable_cmd = DeclareLaunchArgument(
        "enable",
        default_value="True",
        description="Wheter to start darknet enabled")

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.30",
        description="Minimum probability of a detection to be published")

    publish_result_image = LaunchConfiguration("publish_result_image")
    publish_result_image_cmd = DeclareLaunchArgument(
        "publish_result_image",
        default_value="True",
        description="Whether to publish result images with detections")

    show_debug_image = LaunchConfiguration("show_debug_image")
    show_debug_image_cmd = DeclareLaunchArgument(
        "show_debug_image",
        default_value="True",
        description="Whether show image with the detections")

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/image_raw",
        description="Name of the input image topic")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="mmdetection",
        description="Namespace for the nodes")

    #
    # NODES
    #
    detector_node_cmd = Node(
        package="mmdetection_ros",
        executable="mmdetection_node",
        name="mmdetection_node",
        namespace=namespace,
        parameters=[{"config": network_config,
                     "weights": weights,
                     "device": device,
                     "enable": enable,
                     "threshold": threshold}],
        remappings=[("image_raw", input_image_topic)]
    )

    #
    # LAUNCHES
    #
    visualization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mmdetection_bringup_path,
                         "launch",
                         "visualization.launch.py")),
        launch_arguments={
            "publish_result_image": publish_result_image,
            "show_debug_image": show_debug_image,
            "input_image_topic": input_image_topic,
            "namespace": namespace}.items()
    )

    ld = LaunchDescription()

    ld.add_action(network_config_cmd)
    ld.add_action(weights_cmd)
    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(publish_result_image_cmd)
    ld.add_action(show_debug_image_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(detector_node_cmd)
    ld.add_action(visualization_cmd)

    return ld
