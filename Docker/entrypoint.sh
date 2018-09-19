#!/bin/bash

source ~/.bashrc
cd catkin_ws/src
git clone https://github.com/sandeepgogadi/Fetch-ROS-Docker
catkin build
catkin build
source ~/.bashrc
ls ~/.gazebo

/bin/bash
