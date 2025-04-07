# Use Ubuntu 16.04 as the base image
FROM ubuntu:16.04

# Set up the environment to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=kinetic

# Update the package list and install essential packages
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    ca-certificates \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libx11-dev \
    libxrender-dev \
    libxrandr-dev \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    udev \
    usbutils \
    libusb-1.0-0-dev \
    xserver-xorg \
    x11-xserver-utils \
    x11-utils \
    mesa-utils \
    pkg-config \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libxext-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
/usr/local/lib/x86_64-linux-gnu \
/usr/local/lib/x86_64-linux-gnu

COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
ldconfig && \
echo '/usr/local/$LIB/libGL.so.1' >> /etc/ld.so.preload && \
echo '/usr/local/$LIB/libEGL.so.1' >> /etc/ld.so.preload

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Install ROS Kinetic (since Ubuntu 16.04 is compatible with ROS Kinetic)
RUN curl -sSL http://packages.ros.org/ros.key | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && apt-get update \
    && apt-get install -y \
    ros-kinetic-desktop-full \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-rviz \
    ros-kinetic-ros-controllers \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Gazebo 7 (Gazebo 7 is compatible with ROS Kinetic)
RUN apt-get update && apt-get install -y \
    gazebo7 \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages for OpenMANIPULATOR-X
RUN apt-get update && apt-get install -y \
    ros-kinetic-gazebo* \
    ros-kinetic-moveit* \
    ros-kinetic-industrial-core \
    ros-kinetic-dynamixel-sdk \
    ros-kinetic-dynamixel-workbench* \
    ros-kinetic-robotis-manipulator \
    ros-kinetic-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for USB device access
RUN apt-get update && apt-get install -y \
    usb-modeswitch \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Clone the necessary repositories
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git /root/catkin_ws/src/open_manipulator
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git /root/catkin_ws/src/open_manipulator_msgs
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git /root/catkin_ws/src/open_manipulator_simulations
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git /root/catkin_ws/src/open_manipulator_dependencies
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git /root/catkin_ws/src/DynamixelSDK
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench.git /root/catkin_ws/src/dynamixel-workbench
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git /root/catkin_ws/src/dynamixel-workbench-msgs
RUN git clone https://github.com/shailjadav/open_manipulator_friends.git /root/catkin_ws/src/open_manipulator_friends
RUN git clone https://github.com/zang09/open_manipulator_6dof_simulations.git /root/catkin_ws/src/open_manipulator_6dof_simulations
RUN git clone https://github.com/zang09/open_manipulator_6dof_application.git /root/catkin_ws/src/open_manipulator_6dof_application
RUN git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git /root/catkin_ws/src/robotis_manipulator
RUN git clone https://github.com/DarioRepoRuler/open_manipulator_6dof_controls.git /root/catkin_ws/src/open_manipulator_6dof_controls
RUN git clone -b kinetic-devel https://github.com/ros-perception/ar_track_alvar.git /root/catkin_ws/src/ar_track_alvar
RUN git clone https://github.com/shailjadav/advanced_robotics_control.git /root/catkin_ws/src/advanced_robotics_control
RUN git clone https://github.com/DarioRepoRuler/om_position_controller.git /root/catkin_ws/src/om_position_controller

RUN apt-get update
RUN apt-get install -y ros-kinetic-realsense2-camera

# Add user for accessing USB devices
RUN groupadd -r docker && usermod -aG docker root

# Build the workspace
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /root/catkin_ws && catkin_make"

# Set up environment for running GUI applications
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0
RUN apt-get update && apt-get install -y x11-apps && rm -rf /var/lib/apt/lists/*

# Source the ROS environment by default
CMD ["bash", "-c", "source /opt/ros/kinetic/setup.bash && /bin/bash"]
