#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning a Ego Vehicle in ROS

Two modes are available:
- spawn at random Carla Spawnpoint
- spawn at the pose read from ROS topic /initialpose

Whenever a pose is received via /initialpose, the vehicle gets respawned at that
position. If no /initialpose is set at startup, a random spawnpoint is used.

/initialpose might be published via RVIZ '2D Pose Estimate" button.
"""
import os
from abc import abstractmethod
from distutils.spawn import spawn
# from cmath import inf


import os
from pdb import Restart
import sys
import random
import math
import json
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from carla_msgs.msg import CarlaWorldInfo
from std_msgs.msg import Int64
import threading
import carla
import glob
import numpy as np
import time
import argparse
import logging
from carla import VehicleLightState as vls
import easydict
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent


secure_random = random.SystemRandom()

try:
    sys.path.append(glob.glob("/home/autoware/carla_ws/src/carla-autoware-agent/agent/config/sensors.json"))
    sys.path.append(glob.glob('/home/autoware/PythonAPI/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

class ResetCarla(object):
    def __init__(self):
        self.client = carla.Client('192.168.0.121', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()

    def reset(self):
        self.client.reload_world()


