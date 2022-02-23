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

from abc import abstractmethod

import os
from pdb import Restart
import sys
import random
import math
import json
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from carla_msgs.msg import CarlaWorldInfo
from std_msgs.msg import Int64
import threading
import carla
import glob
secure_random = random.SystemRandom()

try:
    sys.path.append(glob.glob("/home/autoware/carla_ws/src/carla-autoware-agent/agent/config/sensors.json"))
except IndexError:
    pass

# ==============================================================================
# -- CarlaEgoVehicle ------------------------------------------------------------
# ==============================================================================


class OtherSensorManager(object):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """
    def __init__(self):
        rospy.init_node('Other_sensors', anonymous=True)
        # self.pub = rospy.Publisher('fault_flag', Int64, queue_size=1)
        self.rate = rospy.Rate(50)
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', 2000)
        self.timeout = rospy.get_param('/carla/timeout', 10)
        self.sensor_definition_file = "/home/autoware/carla_ws/src/carla-autoware-agent/agent/config/sensors.json"
        self.world = None
        self.player = None
        self.player_created = False
        self.sensor_actors = []
        self.actor_filter = rospy.get_param('~vehicle_filter', 'vehicle.*')
        self.actor_spawnpoint = None
        self.faultflag = 0
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.faultId = 4

    def other_sensor_creation(self,ego_id):
        # --------------
        # Add collision sensor to ego vehicle. 
        # --------------

        col_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        col_location = carla.Location(0,0,0)
        col_rotation = carla.Rotation(0,0,0)
        col_transform = carla.Transform(col_location,col_rotation)
        ego_col = self.world.spawn_actor(col_bp,col_transform,attach_to=self.player, attachment_type=carla.AttachmentType.Rigid)
        ego_col.listen(lambda colli: self.col_callback(colli))

        # --------------
        # Add Lane invasion sensor to ego vehicle. 
        # --------------

        lane_bp = self.world.get_blueprint_library().find('sensor.other.lane_invasion')
        lane_location = carla.Location(0,0,0)
        lane_rotation = carla.Rotation(0,0,0)
        lane_transform = carla.Transform(lane_location,lane_rotation)
        ego_lane = self.world.spawn_actor(lane_bp,lane_transform,attach_to=self.player, attachment_type=carla.AttachmentType.Rigid)
        ego_lane.listen(lambda lane: self.lane_callback(lane))

        # --------------
        # Add Obstacle sensor to ego vehicle. 
        # --------------

        obs_bp = self.world.get_blueprint_library().find('sensor.other.obstacle')
        obs_bp.set_attribute("only_dynamics",str(True))
        obs_location = carla.Location(0,0,0)
        obs_rotation = carla.Rotation(0,0,0)
        obs_transform = carla.Transform(obs_location,obs_rotation)
        ego_obs = self.world.spawn_actor(obs_bp,obs_transform,attach_to=self.player, attachment_type=carla.AttachmentType.Rigid)
        ego_obs.listen(lambda obs: self.obs_callback(obs))

    def pubflag(self):    
        self.world = self.client.get_world()
        ego_id = 0 
        while 1:
            actorlist = self.world.get_actors().filter('vehicle.toyota.prius')                
            for actor in actorlist:
                ego_id = actor.id
                ego_candidate = self.world.get_actor(ego_id)
                role_name = ego_candidate.attributes['role_name']
                if role_name == 'ego_vehicle':
                    print("ego id : {}".format(ego_id))
                    real_ego = ego_id
            if ego_id == 0:
                pass
            else:
                print("got ego id")
                break
        self.other_sensor_creation(real_ego)  
        while not rospy.is_shutdown():         
            self.rate.sleep()
    
    def col_callback(colli):
        rospy.loginfo("Collision detected:\n"+str(colli)+'\n')
    
    def lane_callback(lane):
        rospy.loginfo("Lane invasion detected:\n"+str(lane)+'\n')

    def obs_callback(obs):
        rospy.loginfo("Obstacle detected:\n"+str(obs)+'\n')


    @abstractmethod
    def sensors(self):
        """
        return a list of sensors attached
        """
        return []

    def destroy(self):
        """
        destroy the current ego vehicle and its sensors
        """
        for i, _ in enumerate(self.sensor_actors):
            if self.sensor_actors[i] is not None:
                self.sensor_actors[i].destroy()
                self.sensor_actors[i] = None
        self.sensor_actors = []

        if self.player and self.player.is_alive:
            self.player.destroy()
        self.player = None

    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for world info!")
            sys.exit(1)

        rospy.loginfo("CARLA world available. Spawn Fault sensor...")
                    
        try:
            self.pubflag()     
        except rospy.ROSInterruptException:
            pass

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    other_sensor = OtherSensorManager()
    try:        
        other_sensor.run()
    finally:
        if other_sensor is not None:
            other_sensor.destroy()


if __name__ == '__main__':
    main()