FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Install base GUI and graphics libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    locales \
    ca-certificates \
    mesa-utils \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libegl1 \
    libgles2 \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Setup ROS 2 Jazzy repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list

# Install RViz2, Nav2 plugins, robot state publisher, joint state publisher, and tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-ros2cli \
    ros-jazzy-ros2topic \
    ros-jazzy-ros2node \
    ros-jazzy-ros2service \
    && curl -sSL https://github.com/PlotJuggler/plotjuggler_bridge/releases/download/0.10.0/ros-jazzy-pj-bridge_0.10.0-0noble_amd64.deb -o /tmp/ros-jazzy-pj-bridge.deb \
    && apt-get install -y --no-install-recommends /tmp/ros-jazzy-pj-bridge.deb \
    && rm -f /tmp/ros-jazzy-pj-bridge.deb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Configure WSLg hardware acceleration library path
RUN echo "/usr/lib/wsl/lib" > /etc/ld.so.conf.d/ld.wsl.conf && ldconfig

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["rviz2"]
