FROM jeguzzi/ros:kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
   ros-kinetic-dynamic-reconfigure \
   ros-kinetic-tf \
   ros-kinetic-angles \
   && rm -rf /var/lib/apt/lists/*

COPY hl_navigation src/hl_navigation
COPY hl_navigation_msgs src/hl_navigation_msgs
COPY hl_navigation_launch src/hl_navigation_launch

RUN catkin build
