# Use the official Ubuntu 20.04 as the base image
FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

# Set non-interactive mode during installation
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install essential packages
RUN apt-get update && apt-get install -y \
    sudo \
    gnupg \
    lsb-release \
    curl

# Add the ROS Noetic repository
RUN curl -sSL http://packages.ros.org/ros.key | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full


# Setup environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

docker build -t ros-gazebo-nvidia:20.04 .

# Install Gazebo
# RUN apt-get install -y gazebo9

# # Install additional ROS packages
# RUN apt-get install -y \
#     ros-noetic-navigation \
#     ros-noetic-map-server \
#     ros-noetic-move-base \
#     ros-noetic-amcl \
#     ros-noetic-gmapping \
#     ros-noetic-gazebo-ros-control \
#     ros-noetic-joint-state-publisher \
#     ros-noetic-joint-state-controller \
#     ros-noetic-effort-controllers \
#     ros-noetic-velocity-controllers \
#     ros-noetic-joy \
#     ros-noetic-yocs-* \
#     ros-noetic-kobuki* \
#     ros-noetic-depthimage-to-laserscan \
#     ros-noetic-turtlebot-navigation \
#     ros-noetic-openni2-launch \
#     ros-noetic-urg-node

# Fix broken dependencies
RUN apt-get install -y --fix-broken

# Clean up
RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/*

# Expose Gazebo default port
EXPOSE 11345

# Start a shell by default
CMD ["bash"]
