import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Declare launch arguments
    uav_name_arg = DeclareLaunchArgument(
        "uav_name",
        default_value=EnvironmentVariable("UAV_NAME", default_value="uav"),
        description="UAV name"
    )

    fcu_horizontal_frame_arg = DeclareLaunchArgument(
        "fcu_horizontal_frame",
        default_value=LaunchConfiguration("uav_name") + "/fcu_untilted",
        description="FCU horizontal frame ID"
    )

    # Get package share directory
    mrs_bumper_dir = get_package_share_directory("mrs_bumper")

    # Create bumper node
    bumper_node = Node(
        package="mrs_bumper",
        executable="mrs_bumper_node",
        name="bumper",
        output="screen",
        namespace=LaunchConfiguration("uav_name"),
        parameters=[
            os.path.join(mrs_bumper_dir, "config/realworld.yaml"),
            {
                "uav_name": LaunchConfiguration("uav_name"),
                "path_to_mask": LaunchConfiguration("mask_filename"),
                "frame_id": LaunchConfiguration("fcu_horizontal_frame"),
            }
        ],
        remappings=[
            # Laser rangefinder topics
            ("lidar1d_down_in", "hw_api/distance_sensor"),
            ("lidar1d_up_in", "garmin_up/range"),
            # Other input topics
            ("depthmap_in", "front_rgbd/aligned_depth_to_color/image_raw"),
            ("depth_cinfo_in", "front_rgbd/aligned_depth_to_color/camera_info"),
            ("lidar3d_in", "/livox/lidar"),
            ("lidar2d_in", "rplidar/scan"),
            # Output topics
            ("obstacle_sectors_out", "~obstacle_sectors"),
        ]
    )

    # Histogram displayer node
    histogram_node = Node(
        package="mrs_bumper",
        executable="histogram_displayer",
        name="histogram_displayer",
        output="screen",
    )

    return LaunchDescription([
        uav_name_arg,
        fcu_horizontal_frame_arg,
        bumper_node,
    ])
