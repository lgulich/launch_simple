FROM ros:humble

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    python3-pip \
    ros-humble-composition \
    ros-humble-demo-nodes-cpp \
    ros-humble-launch-testing \
    ros-humble-launch-pytest \
    ros-humble-rclcpp-components \
    ros-humble-turtlesim

RUN mkdir -p /workspace/src
WORKDIR /workspace

ENV SHELL /bin/bash

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /workspace/install/setup.bash' >> ~/.bashrc
ENTRYPOINT /bin/bash
