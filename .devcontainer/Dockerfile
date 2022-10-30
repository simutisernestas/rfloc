FROM ros:noetic-ros-core-focal

# TODO: if following fails : )
# RUN echo keyboard-configuration keyboard-configuration/layout select 'English (US)' | sudo debconf-set-selections
# RUN echo keyboard-configuration keyboard-configuration/layoutcode select 'us' | sudo debconf-set-selections 

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-noetic-vrpn-client-ros git python3-catkin-tools python3-vcstool make \
      wget ros-noetic-desktop-full python3-pip \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN pip install pyserial

# TODO: source ~/asta_ws/vrpn_ws/devel/setup.bash add this to /ros_entrypoint.sh : )
# and maybe add ros_entrypoint.sh to ~/.bashrc : )
# ENTRYPOINT ["/bin/bash", "source /ros_entrypoint.sh"]