

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

### Text Markers
`rosrun applications marker_text_demo.py`

`rosrun rviz rviz -d /catkin_ws/src/Fetch-ROS-Docker/rviz/marker.rviz`

### Path Markers
`rosrun applications marker_path_demo.py`

`rosrun rviz rviz -d /catkin_ws/src/Fetch-ROS-Docker/rviz/marker.rviz`

`rosrun applications keyboard_teleop.py`

### Interactive Markers
`rosrun applications interactive_marker_demo.py`

`rosrun rviz rviz -d /catkin_ws/src/Fetch-ROS-Docker/rviz/interactive_marker.rviz`

### Driving with Odometry
`rosrun applications base_demo.py move 0.1`

`rosrun applications base_demo.py rotate 30`

### Building a Map
`roslaunch applications build_map.launch`

`rosrun map_server map_saver -f /catkin_ws/src/Fetch-ROS-Docker/maps/playground`

### Sending navigation goals
`roslaunch applications nav_rviz.launch`

### Saving navigation goals
`rosrun map_annotator annotator.py`

### Cartesian space manipulation
`rosrun applications torso_demo.py 0.4`

`roslaunch fetch_moveit_config move_group.launch`

`rosrun applications cart_arm_demo.py`

### Checking for reachability
#### Check the plan
`rosrun applications check_cart_pose.py plan 0.5 0 1`

`rosrun applications check_cart_pose.py plan 1 0 1`

#### IK
`rosrun applications check_cart_pose.py ik 0.5 0 1`

### Obstacle Avoidance

`rosrun rviz rviz -d /catkin_ws/src/Fetch-ROS-Docker/rviz/arm_obstacle_avoidance.rviz`

`rosrun applications arm_obstacle_demo.py`
