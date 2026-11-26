# ROS 2 Jazzy with everything the H-3215 packages need (in h3215/hardware, h3215/software/control
# and h3215/software/core), for building and testing them on a PC that doesn't run Ubuntu 24.04.
# The workspace is /ws, with the repository mounted at /h3215 and linked into /ws/src, as
# scripts/setup_ros.sh does on the Pi. From the repository folder:
#   docker build -t h3215-ros h3215/software/control
#   docker run --rm -it --user $(id -u):$(id -g) -v "$PWD:/h3215" h3215-ros
# Inside: colcon build, colcon test and the launch files, see docs/pages/software/ros.md.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-ament-cmake-gtest \
      ros-jazzy-realsense2-camera \
      python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Writable by any user, so the build output belongs to whoever runs the container
RUN mkdir -p /ws/src && ln -s /h3215 /ws/src/H-3215 && chmod -R a+rwX /ws
ENV HOME=/tmp
WORKDIR /ws
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /etc/bash.bashrc
CMD ["bash"]
