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
        arguments=['1.30', '0.05', '0.70', '1.6', '0', '-2.3', 'base_link', 'camera_link'],  # XYZ，RPY ; red yaw, green pitch, blue roll

        name='static_tf_node'
    )
    #'-2.35', '0', '1.76' '1.4', '0', '-2.3'
    ld.add_action(vision_node)
    ld.add_action(detect_node)
    ld.add_action(cam_to_real_node)
    ld.add_action(tf_broadcast_node)
    ld.add_action(static_tf)
    
    return ld
