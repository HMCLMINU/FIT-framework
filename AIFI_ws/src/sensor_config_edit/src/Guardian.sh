#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/autoware/.bashrc
source /home/autoware/AIFI_ws/devel/setup.bash

gnome-terminal -e "roslaunch sensor_config_edit sensor_config_edit.launch"
sleep 2
gnome-terminal --working-directory=/home/autoware/AIFI_ws/src/sensor_config_edit/src/bagfile -e "./bag.sh"
