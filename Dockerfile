FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

# Set non-interactive mode during installation
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install essential packages
RUN apt-get update && apt-get install -y \
    sudo \
    gnupg \
    git \
    lsb-release \
    curl

# Set up the ROS keys
RUN curl -sSL http://packages.ros.org/ros.key | apt-key add -
# Add the ROS Noetic repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

# Install Gazebo
RUN apt-get update && apt-get install -y \
    gazebo11

RUN apt-get update && apt-get install -y \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator

# Setup environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Fix broken dependencies
RUN apt-get install -y --fix-broken

# Clean up
RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/*

# Expose Gazebo default port
EXPOSE 11345

# Start a shell by default
CMD ["bash"]
