FROM ros:humble
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

# RUN apt-get update && apt-get install -y \
#    ros-humble-angles \
#    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /ros_ws/src

COPY hl_navigation /ros_ws/src/hl_navigation
COPY hl_navigation_msgs /ros_ws/src/hl_navigation_msgs

RUN cd /ros_ws \
    && source /ros_entrypoint.sh \
    && colcon build --merge-install --install-base /opt/ros/humble \
    && rm -r /ros_ws/build /ros_ws/log
