FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    git curl build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /workspaces/ros2_ws/src
WORKDIR /workspaces/ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
