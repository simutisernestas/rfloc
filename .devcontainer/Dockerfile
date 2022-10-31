FROM ros:noetic-ros-core-focal

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-noetic-vrpn-client-ros git python3-catkin-tools python3-vcstool make \
      wget ros-noetic-desktop-full python3-pip build-essentials netbase \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN pip install pyserial
