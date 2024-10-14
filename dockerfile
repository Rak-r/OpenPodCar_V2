# Use ARG for flexibility to choose ROS2 distribution
# Choose between Humble or Iron
ARG FROM_IMAGE=ros:humble
FROM $FROM_IMAGE

# Set the workspace path as an argument
ARG OVERLAY_WS=/opt/ros/OpenPodCar_V2

# Specify the Gazebo version to install (fortress, garden, harmonic)
ARG GZ_VERSION=fortress

# Update and install necessary tools, including Gz dependencies
RUN apt-get update && apt-get install -y \
    lsb-release wget gnupg curl \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo using OSRF's repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y "gz-${GZ_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS dependencies and useful utilities
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-desktop \
    emacs htop byobu python3-pip less \
    && rm -rf /var/lib/apt/lists/*

# Install specific setuptools version for colcon
RUN pip install --default-timeout=100 setuptools==58.2.0

# Set the working directory to the custom ROS workspace
WORKDIR $OVERLAY_WS/src

# Clone your ROS2 workspace (from your GitHub repo)
# RUN git clone https://github.com/Rak-r/OpenPodCar_V2.git .
COPY src/ .
# Move up to the workspace directory and install dependencies using rosdep
WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace using colcon
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install

# Ensure the ROS environment is properly sourced in the entrypoint
ENV OVERLAY_WS=$OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# Set the default entrypoint to source the ROS environment
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command to keep the container running
CMD ["/bin/bash"]
