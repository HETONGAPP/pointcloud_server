import launch
import launch_ros.actions

def generate_launch_description():
    service1 = launch_ros.actions.Node(
        package='pointcloud_server',
        executable='web_start',
        name='launch'
    )

    service2 = launch_ros.actions.Node(
        package='pointcloud_server',
        executable='web_stop',
        name='launch_stop'
    )

    #service3 = launch_ros.actions.Node(
    #    package='pointcloud_server',
    #    executable='launch_realsense',
    #    name='realsense_camera'
    #)

    return launch.LaunchDescription([
        service1,
        service2
        #service3
    ])
