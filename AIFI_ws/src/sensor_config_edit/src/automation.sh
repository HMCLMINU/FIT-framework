#!/bin/bash
source /home/autoware/.bashrc
source /opt/ros/melodic/setup.bash
source /home/autoware/AIFI_ws/devel/setup.bash

gnome-terminal -e "sshpass -p hmc2020 ssh -t hmcl@192.168.0.121 './FIT_automation.sh'"
sleep 5
gnome-terminal -e "roslaunch carla_autoware_agent carla_autoware_agent.launch"
# sleep 10
# gnome-terminal -e "roslaunch sensor_config_edit sensor_config_edit.launch"