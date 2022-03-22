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

# ==============================================================================
# -- CarlaEgoVehicle ------------------------------------------------------------
# ==============================================================================


class SensorFaultManager(object):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """
    def __init__(self):
        rospy.init_node('FIT_test', anonymous=True)
        self.pub = rospy.Publisher('fault_flag', Int64, queue_size=1)
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
        self.client = carla.Client('192.168.0.121', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.faultId = 1

    def fault_sensor_creation(self,ego_id):
        """
        (Re)spawns the vehicle

        Either at a given actor_spawnpoint or at a random Carla spawnpoint

        :return:
        """
        self.player_created = True
        
        if self.player_created:
            # Read sensors from file
            print(self.sensor_definition_file)
            if not os.path.exists(self.sensor_definition_file):
                raise RuntimeError(
                    "Could not read sensor-definition from {}".format(self.sensor_definition_file))
            json_sensors = None
            with open(self.sensor_definition_file) as handle:
                json_sensors = json.loads(handle.read())

            # Set up the fault sensors
            # To do : 
            # Lidar/IMU/GPS fault sensor creation
            if self.faultId == 1:
                self.sensor_actors = self.setupCameraFault(json_sensors["sensors"],ego_id)
            elif self.faultId == 2:
                self.sensor_actors = self.setupLidarFault(json_sensors["sensors"],ego_id)
            elif self.faultId == 3:
                self.sensor_actors = self.setupImuFault(json_sensors["sensors"],ego_id)
            elif self.faultId == 4:
                self.sensor_actors = self.setupGnssFault(json_sensors["sensors"],ego_id)
            else:
                rospy.loginfo("No fault sensor")

            self.faultflag = 1

    def pubflag(self):    
        self.world = self.client.get_world()
        # settings = self.world.get_settings()
        # settings.synchronous_mode = True
        # self.world.apply_settings(settings)
        ego_id = 0 
        while 1:
            actorlist = self.world.get_actors().filter('vehicle.toyota.prius')
            # sensorlist = self.world.get_actors().filter('sensor.camera.rgb')
            # for sensor in sensorlist:
            #     print("list :{}".format(sensor))
            #     sensor_id = sensor.id
            #     print("sensor id: {}".format(sensor_id))
            #     k = self.world.get_actor(sensor_id)
            #     print("attributes:{}".format(k.set_attributes()))                  
            for actor in actorlist:
                ego_id = actor.id
                ego_candidate = self.world.get_actor(ego_id)
                role_name = ego_candidate.attributes['role_name']
                # print("ego candidate : {}".format(role_name))
                if role_name == 'ego_vehicle':
                    print("ego id : {}".format(ego_id))
                    real_ego = ego_id
            if ego_id == 0:
                # print("ego id is none")
                pass
            else:
                print("got ego id")
                break
        self.fault_sensor_creation(real_ego)  
        while not rospy.is_shutdown():         
            # print("{}".format(self.world.get_actors()))                                   
            # rospy.loginfo(self.faultflag)
            self.pub.publish(self.faultflag)
            self.rate.sleep()

    def setupCameraFault(self, sensors, ego_id):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param sensors: list of sensors
        :return:
        """
        # for actor in wordl.get_actors():
        #     if actor.attributes.get('role_name') == 
        actors = []
        bp_library = self.world.get_blueprint_library()
        sensor_names = []
        for sensor_spec in sensors:
            try:
                sensor_type = str(sensor_spec.pop("type"))
                if (sensor_type == "sensor.camera.rgb"):
                    sensor_id = str("FIT")+str(sensor_spec.pop("id"))
                    if sensor_id == 'FITfront':
                        sensor_name = sensor_type + "/" + sensor_id
                        if sensor_name in sensor_names:
                            rospy.logfatal(
                                "Sensor rolename '{}' is only allowed to be used once.".format(
                                    sensor_spec['id']))
                            raise NameError(
                                "Sensor rolename '{}' is only allowed to be used once.".format(
                                    sensor_spec['id']))
                        sensor_names.append(sensor_name)

                        sensor_location = carla.Location(x=sensor_spec.pop("x"),
                                                        y=sensor_spec.pop("y"),
                                                        z=sensor_spec.pop("z"))
                        sensor_rotation = carla.Rotation(
                            pitch=sensor_spec.pop('pitch', 0.0),
                            roll=sensor_spec.pop('roll', 0.0),
                            yaw=sensor_spec.pop('yaw', 0.0))
                        sensor_transform = carla.Transform(sensor_location, sensor_rotation)

                        bp = bp_library.find(sensor_type)
                        bp.set_attribute('role_name', sensor_id)
                        for attribute, value in sensor_spec.items():
                            if str(attribute) == "lens_flare_intensity":
                                bp.set_attribute(str(attribute), str(20)) # lens flare fault
                            else:
                                bp.set_attribute(str(attribute), str(value))
                        rospy.loginfo("FIT Camera {} created".format(sensor_id))
                        break

            except KeyError as e:
                rospy.logfatal(
                    "Sensor will not be spawned, the mandatory attribute {} is missing"
                    .format(e))
                raise e

            except IndexError as e:
                rospy.logfatal(
                    "Sensor {} will not be spawned, {}".format(sensor_name, e))
                raise e

        # create sensor
        sensor = self.world.spawn_actor(bp, sensor_transform,
                                        attach_to=self.world.get_actor(ego_id))
        actors.append(sensor)
            
        return actors
    
    def setupLidarFault(self, sensors, ego_id):
        actors = []
        bp_library = self.world.get_blueprint_library()
        sensor_names = []
        for sensor_spec in sensors:
            try:
                sensor_type = str(sensor_spec.pop("type"))
                # rospy.loginfo("{}".format(sensor_type))
                if (sensor_type == "sensor.lidar.ray_cast"):
                    sensor_id = str("FIT")+str(sensor_spec.pop("id"))
                    if sensor_id == 'FITlidar1':
                        sensor_name = sensor_type + "/" + sensor_id
                        if sensor_name in sensor_names:
                            rospy.logfatal(
                                "Sensor rolename '{}' is only allowed to be used once.".format(
                                    sensor_spec['id']))
                            raise NameError(
                                "Sensor rolename '{}' is only allowed to be used once.".format(
                                    sensor_spec['id']))
                        sensor_names.append(sensor_name)

                        sensor_location = carla.Location(x=sensor_spec.pop("x"),
                                                        y=sensor_spec.pop("y"),
                                                        z=sensor_spec.pop("z"))
                        sensor_rotation = carla.Rotation(
                            pitch=sensor_spec.pop('pitch', 0.0),
                            roll=sensor_spec.pop('roll', 0.0),
                            yaw=sensor_spec.pop('yaw', 0.0))
                        sensor_transform = carla.Transform(sensor_location, sensor_rotation)

                        bp = bp_library.find(sensor_type)
                        bp.set_attribute('role_name', sensor_id)
                        # Fault creation
                        for attribute, value in sensor_spec.items():
                            if str(attribute) == "atmosphere_attenuation_rate":
                                bp.set_attribute(str(attribute), str(0.004)) # intensity loss per meter
                            elif str(attribute) == "dropoff_general_rate":
                                bp.set_attribute(str(attribute), str(0.004)) # randomly dropped point fault
                            elif str(attribute) == "noise_stddev":
                                bp.set_attribute(str(attribute), str(0)) # noise fault
                            else:
                                bp.set_attribute(str(attribute), str(value))
                        rospy.loginfo("FIT Lidar {} created".format(sensor_id))
                        break
                

            except KeyError as e:
                rospy.logfatal(
                    "Sensor will not be spawned, the mandatory attribute {} is missing"
                    .format(e))
                raise e

            except IndexError as e:
                rospy.logfatal(
                    "Sensor {} will not be spawned, {}".format(sensor_name, e))
                raise e

        # create sensor
        sensor = self.world.spawn_actor(bp, sensor_transform,
                                        attach_to=self.world.get_actor(ego_id))
        actors.append(sensor)

        return actors

    def setupImuFault(self, sensors, ego_id):
        actors = []
        bp_library = self.world.get_blueprint_library()
        sensor_names = []
        for sensor_spec in sensors:
            try:
                sensor_type = str(sensor_spec.pop("type"))
                if (sensor_type == "sensor.other.imu"):
                    sensor_id = str("FIT")+str(sensor_spec.pop("id"))
                    if sensor_id == 'FITimu1':
                        sensor_name = sensor_type + "/" + sensor_id
                        if sensor_name in sensor_names:
                            rospy.logfatal(
                                "Sensor rolename '{}' is only allowed to be used once.".format(
                                    sensor_spec['id']))
                            raise NameError(
                                "Sensor rolename '{}' is only allowed to be used once.".format(
                                    sensor_spec['id']))
                        sensor_names.append(sensor_name)

                        sensor_location = carla.Location(x=sensor_spec.pop("x"),
                                                        y=sensor_spec.pop("y"),
                                                        z=sensor_spec.pop("z"))
                        sensor_rotation = carla.Rotation(
                            pitch=sensor_spec.pop('pitch', 0.0),
                            roll=sensor_spec.pop('roll', 0.0),
                            yaw=sensor_spec.pop('yaw', 0.0))
                        sensor_transform = carla.Transform(sensor_location, sensor_rotation)

                        bp = bp_library.find(sensor_type)
                        bp.set_attribute('role_name', sensor_id)
                        # Fault creation
                        for attribute, value in sensor_spec.items():
                            if str(attribute) == "noise_accel_stddev_x":
                                bp.set_attribute(str(attribute), str(0)) # x accel noise fault
                            elif str(attribute) == "noise_accel_stddev_y":
                                bp.set_attribute(str(attribute), str(0)) # y accel noise fault
                            elif str(attribute) == "noise_gyro_bias_x":
                                bp.set_attribute(str(attribute), str(0)) # x gyro noise bias fault
                            elif str(attribute) == "noise_gyro_bias_y":
                                bp.set_attribute(str(attribute), str(0)) # y gyro noise bias fault
                            else:
                                bp.set_attribute(str(attribute), str(value))
                        rospy.loginfo("FIT IMU {} created".format(sensor_id))
                        break

            except KeyError as e:
                rospy.logfatal(
                    "Sensor will not be spawned, the mandatory attribute {} is missing"
                    .format(e))
                raise e

            except IndexError as e:
                rospy.logfatal(
                    "Sensor {} will not be spawned, {}".format(sensor_name, e))
                raise e

            # create sensor
        sensor = self.world.spawn_actor(bp, sensor_transform,
                                        attach_to=self.world.get_actor(ego_id))
        actors.append(sensor)

        return actors

    def setupGnssFault(self, sensors, ego_id):
        actors = []
        bp_library = self.world.get_blueprint_library()
        sensor_names = []
        for sensor_spec in sensors:
            try:
                sensor_type = str(sensor_spec.pop("type"))
                if (sensor_type == "sensor.other.gnss"):
                    sensor_id = str("FIT")+str(sensor_spec.pop("id"))
                    if sensor_id == 'FITgnss1':
                        sensor_name = sensor_type + "/" + sensor_id
                        if sensor_name in sensor_names:
                            rospy.logfatal(
                                "Sensor rolename '{}' is only allowed to be used once.".format(
                                    sensor_spec['id']))
                            raise NameError(
                                "Sensor rolename '{}' is only allowed to be used once.".format(
                                    sensor_spec['id']))
                        sensor_names.append(sensor_name)

                        sensor_location = carla.Location(x=sensor_spec.pop("x"),
                                                        y=sensor_spec.pop("y"),
                                                        z=sensor_spec.pop("z"))
                        sensor_rotation = carla.Rotation(
                            pitch=sensor_spec.pop('pitch', 0.0),
                            roll=sensor_spec.pop('roll', 0.0),
                            yaw=sensor_spec.pop('yaw', 0.0))
                        sensor_transform = carla.Transform(sensor_location, sensor_rotation)

                        bp = bp_library.find(sensor_type)
                        bp.set_attribute('role_name', sensor_id)
                        # Fault creation
                        for attribute, value in sensor_spec.items():
                            if str(attribute) == "noise_lat_bias":
                                bp.set_attribute(str(attribute), str(0)) # noise lat bias fault
                            elif str(attribute) == "noise_lat_stddev":
                                bp.set_attribute(str(attribute), str(0)) # noise lat fault
                            elif str(attribute) == "noise_lon_bias":
                                bp.set_attribute(str(attribute), str(0)) # noise lon bias fault
                            elif str(attribute) == "noise_lon_stddev":
                                bp.set_attribute(str(attribute), str(0)) # noise lon fault
                            else:
                                bp.set_attribute(str(attribute), str(value))
                        rospy.loginfo("FIT GNSS {} created".format(sensor_id))
                        break

            except KeyError as e:
                rospy.logfatal(
                    "Sensor will not be spawned, the mandatory attribute {} is missing"
                    .format(e))
                raise e

            except IndexError as e:
                rospy.logfatal(
                    "Sensor {} will not be spawned, {}".format(sensor_name, e))
                raise e

        # create sensor
        sensor = self.world.spawn_actor(bp, sensor_transform,
                                        attach_to=self.world.get_actor(ego_id))
        actors.append(sensor)

        return actors

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
    sensor_fault = SensorFaultManager()
    try:
        sensor_fault.run()
    finally:
        if sensor_fault is not None:
            sensor_fault.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
