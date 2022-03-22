#!/bin/bash
source /opt/ros/melodic/setup.bash

# roslaunch sensor_config_edit bag.launch
rosbag record /carla/ego_vehicle/vehicle_status\
    
