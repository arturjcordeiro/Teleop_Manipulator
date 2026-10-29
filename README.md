sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt update
sudo apt install librtde librtde-dev

# Test with

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=0.0.0.0   use_mock_hardware:=true launch_rviz:=true   initial_joint_controller:=forward_position_controller
