mkdir ~/local
cd ~/local
git clone https://github.com/fetchrobotics/fetch_ros -b indigo-devel
cp -r fetch_ros/fetch_ikfast_plugin /catkin_ws/src/Fetch-ROS-Docker/
cd /catkin_ws/src/Fetch-ROS-Docker/
catkin build fetch_ikfast_plugin

source ~/.bashrc
mkdir /catkin_ws/src/Fetch-ROS-Docker/fetch_api/launch
mkdir /catkin_ws/src/Fetch-ROS-Docker/fetch_api/config
cd /catkin_ws/src/Fetch-ROS-Docker/fetch_api
roscp fetch_moveit_config move_group.launch launch
roscp fetch_moveit_config planning_context.launch launch
roscp fetch_moveit_config kinematics.yaml config
