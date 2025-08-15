import os

from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # RealSense node
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera_node",
        output="screen",
        parameters=[
            # add overrides here
        ],
    )

    # Attempt to include lucid launch
    
    lucid_launch_path = os.path.join(
        get_package_share_directory("lucid_vision_driver"),
        "launch",
        "test_node_container.launch.py",
    )

    include_lucid = None
    if os.path.exists(lucid_launch_path):
        include_lucid = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lucid_launch_path),
            # If that launch expects arguments, supply like:
            # launch_arguments={"arg_name": "value"}.items()
        )
    else:
        # Will show error in console but still launch Realsense
        missing_msg = LogInfo(
            msg=f"Lucid launch file not found at {lucid_launch_path}; only RealSense will start."
        )
        include_lucid = missing_msg
        

    return LaunchDescription([
        realsense_node,
        include_lucid,
    ])
