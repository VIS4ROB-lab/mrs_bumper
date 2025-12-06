import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Declare launch arguments
    uav_name_arg = DeclareLaunchArgument(
        "uav_name",
        default_value=EnvironmentVariable("UAV_NAME", default_value="uav"),
        description="UAV name"
    )

    run_type_arg = DeclareLaunchArgument(
        "run_type",
        default_value=EnvironmentVariable("RUN_TYPE", default_value="simulation"),
        description="Run type (simulation or realworld)"
    )

    standalone_arg = DeclareLaunchArgument(
        "standalone",
        default_value="true",
        description="Run as standalone node or in a container"
    )

    fcu_frame_arg = DeclareLaunchArgument(
        "fcu_frame",
        default_value=LaunchConfiguration("uav_name") + "/fcu",
        description="FCU frame ID"
    )

    fcu_horizontal_frame_arg = DeclareLaunchArgument(
        "fcu_horizontal_frame",
        default_value=LaunchConfiguration("uav_name") + "/fcu_untilted",
        description="FCU horizontal frame ID"
    )

    custom_config_arg = DeclareLaunchArgument(
        "custom_config",
        default_value="",
        description="Custom config file path"
    )

    ignore_mask_arg = DeclareLaunchArgument(
        "ignore_mask",
        default_value=EnvironmentVariable("IGNORE_MASK", default_value="true"),
        description="Ignore depthmap mask"
    )

    # Get package share directory
    from ament_index_python.packages import get_package_share_directory
    mrs_bumper_dir = get_package_share_directory("mrs_bumper")

    # Create bumper node
    bumper_node = Node(
        package="mrs_bumper",
        executable="mrs_bumper_node",
        name="bumper",
        output="screen",
        namespace=LaunchConfiguration("uav_name"),
        parameters=[
            os.path.join(mrs_bumper_dir, "config", LaunchConfiguration("run_type"), ".yaml"),
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
            ("lidar3d_in", "os_cloud_nodelet/points"),
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
        run_type_arg,
        standalone_arg,
        fcu_frame_arg,
        fcu_horizontal_frame_arg,
        custom_config_arg,
        ignore_mask_arg,
        bumper_node,
    ])
