#!/bin/bash
source /home/autoware/.bashrc
source /opt/ros/melodic/setup.bash
source /home/autoware/AIFI_ws/devel/setup.bash

gnome-terminal -e "roslaunch sensor_config_edit sensor_config_edit.launch"

