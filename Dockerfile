FROM ubuntu:22.04
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

ENV DEBIAN_FRONTEND="noninteractive" TZ="Europe/Zurich"

# sim -> highfive -> libhdf5 -> libboost

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    # libgeos++-dev \
    libeigen3-dev \
    python3-dev \
    python3-pip \
    ament-cmake \
    libhdf5-dev \
    libboost-all-dev \
    python3-numpy \
   && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN pip3 install -U colcon-common-extensions

RUN mkdir -p /ws/src && cd /ws/src \
    && git clone https://github.com/libgeos/geos.git \
    && git clone https://github.com/pybind/pybind11.git \
    && git clone https://github.com/BlueBrain/HighFive.git \
    && git clone https://github.com/jbeder/yaml-cpp.git

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF  --packages-select pybind11

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DYAML_CPP_INSTALL=ON --packages-select YAML_CPP

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DGEOS_BUILD_DEVELOPER=OFF --packages-select GEOS

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DHIGHFIVE_UNIT_TESTS=OFF -DHIGHFIVE_BUILD_DOCS=OFF --packages-select HighFive


RUN apt-get update && apt-get install -y \
    clang \
   && rm -rf /var/lib/apt/lists/*

RUN pip3 install clang==14 git+https://github.com/jeguzzi/pybind11_mkdoc@rst

RUN cd /ws/src && git clone https://github.com/jeguzzi/hl_navigation.git --branch ros2

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select  \
        hl_navigation_core \
        hl_navigation_py \
        hl_navigation_sim \
        hl_navigation_examples \
        hl_navigation_examples_py \
        hl_navigation_demos


