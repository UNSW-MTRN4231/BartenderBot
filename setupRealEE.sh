gnome-terminal -t "DriverServer" -e 'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=false launch_rviz:=false'

sleep 10

gnome-terminal -t "MoveitServer" -e 'ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true description_package:=ur_with_end_effector_description description_file:=ur_with_end_effector.xacro'





