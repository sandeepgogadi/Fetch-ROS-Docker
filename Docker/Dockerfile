FROM osrf/ros:indigo-desktop-full

MAINTAINER Sandeep Gogadi <sandeepgogadi@yahoo.com>

# Replacing shell with bash for later source, catkin build commands
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

### Nvidia Driver
LABEL com.nvidia.volumes.needed="nvidia_driver"
### Nvidia path
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

### Source ROS
RUN echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

### Install essentials
RUN apt-get update && apt-get install -q -y \
    nano wget \
    python-catkin-tools \
    ros-indigo-fetch-* \
    ros-indigo-moveit-* \
    && rm -rf /var/lib/apt/lists/*

### Create workspace
RUN source ~/.bashrc \
    && mkdir -p /catkin_ws/src \
    && cd /catkin_ws \
    && catkin init \
    && catkin config --extend /opt/ros/indigo \
    && catkin build \
    && echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

### Gazebo sources and keys
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

### Update all packages
RUN apt-get update \
    && apt-get -y upgrade

### Gazebo Models
RUN mkdir -p ~/.gazebo/models \
    && hg clone https://bitbucket.org/osrf/gazebo_models ~/.gazebo/models

### Add entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
CMD ["/entrypoint.sh"]
