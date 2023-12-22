FROM ros:humble

ARG USERNAME=DUMMY_USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

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
RUN chown -R $USER_UID:$USER_GID /workspace

ENV SHELL /bin/bash
USER $USERNAME

COPY .devcontainer/entrypoint.sh /entrypoint.sh
ENTRYPOINT /entrypoint.sh
CMD ["/bin/bash"]
