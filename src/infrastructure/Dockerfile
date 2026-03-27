FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Core utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
		ca-certificates \
		curl \
		gnupg2 \
		lsb-release \
		software-properties-common \
		git \
		build-essential \
		cmake \
		gdb \
		vim \
		python3 \
		python3-pip \
		python3-venv \
		&& rm -rf /var/lib/apt/lists/*

# ROS2 Jazzy apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
		-o /usr/share/keyrings/ros-archive-keyring.gpg && \
		echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
		http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
		> /etc/apt/sources.list.d/ros2.list

# ROS2 Jazzy + tooling + dependencies for this project
RUN apt-get update && apt-get install -y --no-install-recommends \
		ros-jazzy-desktop \
		python3-colcon-common-extensions \
		python3-rosdep \
		python3-vcstool \
		python3-argcomplete \
		ros-jazzy-geometry-msgs \
		ros-jazzy-nav-msgs \
		ros-jazzy-sensor-msgs \
		ros-jazzy-std-msgs \
		ros-jazzy-std-srvs \
		ros-jazzy-rclcpp \
		ros-jazzy-rclcpp-action \
		ros-jazzy-rclcpp-components \
		ros-jazzy-launch \
		ros-jazzy-launch-ros \
		ros-jazzy-tf2 \
		ros-jazzy-tf2-ros \
		ros-jazzy-tf2-geometry-msgs \
		ros-jazzy-ament-cmake \
		ros-jazzy-ament-cmake-gtest \
		ros-jazzy-ros-testing \
		ros-jazzy-smacc2 \
		ros-jazzy-smacc2-msgs \
		ros-jazzy-moveit \
		&& rm -rf /var/lib/apt/lists/*

# MAVLink and MAVROS packages (ROS2 Jazzy)
# Use ROS packages on Ubuntu 24.04 because libmavlink-dev may be unavailable.
RUN apt-get update && apt-get install -y --no-install-recommends \
		ros-jazzy-mavlink \
		ros-jazzy-libmavconn \
		ros-jazzy-mavros-msgs \
		ros-jazzy-mavros \
		&& rm -rf /var/lib/apt/lists/*
		
# Workspace setup
RUN mkdir -p /opt/ws/src
WORKDIR /opt/ws

# ROS setup in shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# rosdep init (safe if already initialized)
RUN rosdep init || true
RUN rosdep update || true

CMD ["bash"]
