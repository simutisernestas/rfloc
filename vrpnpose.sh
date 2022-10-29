#!/bin/bash

source /ros_entrypoint.sh
source ~/asta_ws/vrpn_ws/devel/setup.bash
rostopic echo /vrpn_client_node/awww/pose