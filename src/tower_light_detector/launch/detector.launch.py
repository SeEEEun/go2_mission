from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument("fps",          default_value="10"),
        DeclareLaunchArgument("show_debug",   default_value="true"),
        # ROI: set roi_w/roi_h > 0 to enable (crop to tower light area)
        DeclareLaunchArgument("roi_x", default_value="0"),
        DeclareLaunchArgument("roi_y", default_value="0"),
        DeclareLaunchArgument("roi_w", default_value="0"),
        DeclareLaunchArgument("roi_h", default_value="0"),

        Node(
            package="tower_light_detector",
            executable="detector",
            name="tower_light_detector",
            parameters=[{
                "camera_index": LaunchConfiguration("camera_index"),
                "fps":          LaunchConfiguration("fps"),
                "show_debug":   LaunchConfiguration("show_debug"),
                "roi_x": LaunchConfiguration("roi_x"),
                "roi_y": LaunchConfiguration("roi_y"),
                "roi_w": LaunchConfiguration("roi_w"),
                "roi_h": LaunchConfiguration("roi_h"),
            }],
            output="screen",
        ),

        Node(
            package="tower_light_detector",
            executable="mission",
            name="mission_dispatcher",
            output="screen",
        ),
    ])
