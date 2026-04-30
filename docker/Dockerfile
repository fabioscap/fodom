FROM ros:jazzy

# base image setup

# install packages
RUN apt update && \
    apt-get install git libxinerama-dev libxi-dev libxcursor-dev tmux micro -y

# set up workspace
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

RUN apt install ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-plotjuggler-ros ros-dev-tools -y

RUN apt install wget -y && wget https://get.foxglove.dev/desktop/v2.13.0/foxglove-studio-2.13.0-linux-amd64.deb && DEBIAN_FRONTEND=noninteractive apt install ./foxglove-studio-*.deb -y

WORKDIR /root

RUN mkdir -p ros2_ws/src && ln -s /root/sources/ /root/ros2_ws/src/fodom
