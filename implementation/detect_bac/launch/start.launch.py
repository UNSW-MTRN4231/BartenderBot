from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    vision_node = Node(
        package='vision_opencv',
        executable='vision_opencv',
        name='vision_opencv'
    )

    detect_node = Node(
        package='detect_bac',
        executable='detect_node',
        name='detect_node'
    )

    cam_to_real_node = Node(
        package='detect_bac',
        executable='cam_to_real',
        name='cam_to_real'
    )

    tf_broadcast_node = Node(
        package='detect_bac',
        executable='tf_broadcast',
        name='tf_broadcast'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['1.38', '0.08', '0.78', 'M_PI', 'M_PI/4', 'M_PI', 'world', 'camera_frame'],  # XYZ，RPY 

        name='static_tf_node'
    )

    ld.add_action(vision_node)
    ld.add_action(detect_node)
    ld.add_action(cam_to_real_node)
    ld.add_action(tf_broadcast_node)
    ld.add_action(static_tf)
    
    return ld
