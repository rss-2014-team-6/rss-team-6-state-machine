#!/bin/bash

. /opt/ros/hydro/setup.sh

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${HOME}/rss-team-6
export DISPLAY=:0
export ROS_HOSTNAME=workstation
exec "$@"