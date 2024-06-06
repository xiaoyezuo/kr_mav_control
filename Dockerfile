#Docker file for ubuntu 22.04 with ROS humble on Mac M1

FROM ubuntu:jammy

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    make \
    build-essential \
    g++ \
    tmux \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*


# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-core=0.10.0-1* \
    ros-${ROS_DISTRO}-tf2* \
    ros-${ROS_DISTRO}-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

# instal colcon 
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

# Add user
ARG GID=1011
ARG UID=1011
ENV USER xiaoyez
RUN addgroup --gid $GID $USER 
RUN useradd --system --create-home --shell /bin/bash --groups sudo -p "$(openssl passwd -1 ${USER})" --uid $UID --gid $GID $USER

#workspace setup
ENV HOME /home/$USER
RUN mkdir -p $HOME/ros2_ws/src
WORKDIR $HOME/ros2_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"
# WORKDIR ${HOME}/ros2_ws
# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]