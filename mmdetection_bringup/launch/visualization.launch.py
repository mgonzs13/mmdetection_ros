from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    #
    # ARGS
    #
    publish_result_image = LaunchConfiguration("publish_result_image")
    publish_result_image_cmd = DeclareLaunchArgument(
        "publish_result_image",
        default_value="True",
        description="Whether to publish result images with detections")

    show_debug_image = LaunchConfiguration("show_debug_image")
    show_debug_image_cmd = DeclareLaunchArgument(
        "show_debug_image",
        default_value="True",
        description="Whether show image with bounding boxes")

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/image_raw",
        description="Name of the input image topic")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="mmdetection",
        description="Namespace")

    #
    # NODES
    #

    detection_visualizer_cmd = Node(
        package="mmdetection_ros",
        executable="visualization_node",
        name="visualization_node",
        namespace=namespace,
        remappings=[("image_raw", input_image_topic)],
        condition=IfCondition(PythonExpression([publish_result_image]))
    )

    image_view_cmd = Node(
        package="image_view",
        executable="image_view",
        name="image_view",
        namespace=namespace,
        remappings=[("image", "result_image")],
        condition=IfCondition(PythonExpression(
            [show_debug_image, " and ", publish_result_image]))
    )

    ld = LaunchDescription()

    ld.add_action(publish_result_image_cmd)
    ld.add_action(show_debug_image_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(detection_visualizer_cmd)
    ld.add_action(image_view_cmd)

    return ld
