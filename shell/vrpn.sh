#!/bin/bash

source /ros_entrypoint.sh
source ~/asta_ws/vrpn_ws/devel/setup.bash
roslaunch vrpn_client_ros sample.launch server:=192.168.1.129