import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    xml_file_path = '/opt/ros/humble/share/rosbridge_server/launch/rosbridge_websocket_launch.xml'
    web_service = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/mnt/tong/ros2_ws/src/pointcloud_server/launch/launch_web_service.launch.py'
        )
    )

    webrtc_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/mnt/tong/ros2_ws_foxy/src/webrtc_ros/launch/webrtc_ros_server.launch.py'
        )
    )

    return LaunchDescription([
        web_service,
        webrtc_ros,
        IncludeLaunchDescription(
            launch_description_source = xml_file_path,
        )

    ])
    

