

### Fetch Simulator
`roscore`

`roslaunch fetch_gazebo playground.launch`

### Base
`rosrun applications keyboard_teleop.py`

### Gripper
`rosrun applications gripper_demo.py open`

`rosrun applications gripper_demo.py close`

### Torso
`rosrun applications torso_demo.py 0.1`

`rosrun applications torso_demo.py 0.4`

`rosrun applications torso_demo.py 0.0`

### Head
`rosrun applications head_demo.py look_at base_link 1 0 0.3`

`rosrun applications head_demo.py pan_tilt 0 0`

### Arm
`rosrun applications arm_demo.py`

### Joint States
`rosrun applications joint_reader_demo.py`

### Rviz
`rosrun rviz rviz -d /catkin_ws/src/Fetch-ROS-Docker/rviz/default.rviz`
