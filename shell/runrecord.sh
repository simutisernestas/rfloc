#!/bin/bash

source /ros_entrypoint.sh
python3 recordexp.py & rosbag record -a
