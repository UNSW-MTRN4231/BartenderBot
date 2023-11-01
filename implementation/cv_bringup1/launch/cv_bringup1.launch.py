from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace='camera1'
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace='camera2',
            arguments=["--ros-args","-p","video_device:=/dev/video2"],
            remappings=[
                ('/image_raw', '/camera2/image')
            ]
        )
    ])