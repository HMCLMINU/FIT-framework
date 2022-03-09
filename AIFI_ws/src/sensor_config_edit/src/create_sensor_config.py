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
        self.faultId = 4

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

class ScenarioManager(object):
    def __init__(self):
        self.client = carla.Client('192.168.0.121', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.startPosePub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.goalPosePub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.done = False

    def setEgo(self): # set ego state publish
        startpose = PoseWithCovarianceStamped()
        goalpose = PoseStamped()
        now = rospy.get_rostime()
        
        startpose.header.frame_id = "map"
        startpose.header.stamp = now
        startpose.pose.pose.position.x = 400
        startpose.pose.pose.position.y = -275
        startpose.pose.pose.position.z = 0
        startpose.pose.pose.orientation.x = 0
        startpose.pose.pose.orientation.y = 0 
        startpose.pose.pose.orientation.z = 0 
        startpose.pose.pose.orientation.w = 0
        startpose.pose.covariance = np.zeros(36)

        goalpose.header.frame_id = "map"
        goalpose.header.stamp = rospy.Time.now()
        goalpose.pose.position.x = 2
        goalpose.pose.position.y = -296
        goalpose.pose.position.z = 0
        goalpose.pose.orientation.x = 0
        goalpose.pose.orientation.y = 0
        goalpose.pose.orientation.z = 0.7
        goalpose.pose.orientation.w = 0.7

        if now < rospy.Time(1):
            pass
        else:
            # print("now : {}".format(now))
            self.startPosePub.publish(startpose)
            self.goalPosePub.publish(goalpose)
            self.done = True



    def setObstacle(self): # set obstacle state
        # return start_position, goal_position, start_vel

        args = easydict.EasyDict({
            "host" : '192.168.0.121',
            "port" : 2000,
            "number_of_vehicles" : 20,
            "number_of_walkers" : 0,
            "safe" : True,
            "filterv" : 'vehicle.*',
            "filterw" : 'walker.pedestrian.*',
            "tm_port" : 8000,
            "sync" : True,
            "hybrid" : True,
            "seed" : 50,
            "car_lights_on" : False
        })

        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

        vehicles_list = []
        walkers_list = []
        all_id = []
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        synchronous_master = False
        random.seed(args.seed if args.seed is not None else int(time.time()))

        try:
            world = client.get_world()
            traffic_manager = client.get_trafficmanager(args.tm_port)
            traffic_manager.set_global_distance_to_leading_vehicle(1.0)
            if args.hybrid:
                traffic_manager.set_hybrid_physics_mode(True)
            if args.seed is not None:
                traffic_manager.set_random_device_seed(args.seed)


            if args.sync:
                settings = world.get_settings()
                traffic_manager.set_synchronous_mode(True)
                print("synch : {}".format(settings.synchronous_mode))
                if not settings.synchronous_mode:
                    synchronous_master = True
                    settings.synchronous_mode = True
                    settings.fixed_delta_seconds = 0.05
                    world.apply_settings(settings)
                else:
                    synchronous_master = False

            blueprints = world.get_blueprint_library().filter(args.filterv)
            blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)

            if args.safe:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
                blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
                blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('t2')]

            blueprints = sorted(blueprints, key=lambda bp: bp.id)
            ############### set spawn point by location & rotation ###############
            # location : x, y, z
            # rotation : pitch, yaw, roll

            # get spawn points from map
            spawn_points = world.get_map().get_spawn_points()
            # get spawn points from map by 5 meter
            waypoints = client.get_world().get_map().generate_waypoints(distance=5.0)
            # plot road id
            self.plotRoadid()
            ##### Filter road #####
            filtered_waypoints = []
            for waypoint in waypoints:
                if(waypoint.lane_id == 1) and (waypoint.road_id == 1 or waypoint.road_id == 25): # cluster by road_id and lane_id
                    filtered_waypoints.append(waypoint) # add waypoints

            for i in range(len(filtered_waypoints)): # plot added waypoints
                client.get_world().debug.draw_string(filtered_waypoints[i].transform.location, str(i), draw_shadow=False,
                            color=carla.Color(r=0, g=255, b=0), life_time=100000,
                            persistent_lines=True)  

            vehicle_blueprint = client.get_world().get_blueprint_library().filter('model3')[0] # vehicle object
            number_of_spawn_points = len(waypoints)
            # set SV spawn point
            spawn_point_v = filtered_waypoints[7].transform
            print(spawn_point_v)
            spawn_point_v.location.z += 2
            spawn_point_v.rotation.yaw -= 0
            # spawn SV
            vehicle = client.get_world().spawn_actor(vehicle_blueprint, spawn_point_v)
            # set control info of SV
            agent = BehaviorAgent(vehicle, behavior='normal')    
            # set a destination of SV
            destination = filtered_waypoints[20].transform.location
            agent.set_destination(destination)
            client.get_world().debug.draw_string(destination, 'O', draw_shadow=False,
                        color=carla.Color(r=255, g=0, b=0), life_time=100000,
                        persistent_lines=True)
            print("SV set done. \n")
            while True: # control SV
                if agent.done():
                    # agent.set_destination(random.choice(spawn_points).location)
                    # print("The target has been reached, searching for another target")
                    print("The target has been reached, stopping the simulation")
                    break
                vehicle.apply_control(agent.run_step())
        
            
            # if args.number_of_vehicles < number_of_spawn_points:
            #     random.shuffle(spawn_points)
            # elif args.number_of_vehicles > number_of_spawn_points:
            #     msg = 'requested %d vehicles, but could only find %d spawn points'
            #     # logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            #     args.number_of_vehicles = number_of_spawn_points

            # # @todo cannot import these directly.
            # SpawnActor = carla.command.SpawnActor
            # SetAutopilot = carla.command.SetAutopilot
            # SetVehicleLightState = carla.command.SetVehicleLightState
            # FutureActor = carla.command.FutureActor

            # # --------------
            # # Spawn vehicles
            # # --------------
            # batch = []
            # for n, transform in enumerate(spawn_points):
            #     print("n : {}".format(n))
            #     print("transform : {}".format(transform))
            #     if n >= args.number_of_vehicles:
            #         break
            #     blueprint = random.choice(blueprints)
            #     if blueprint.has_attribute('color'):
            #         color = random.choice(blueprint.get_attribute('color').recommended_values)
            #         blueprint.set_attribute('color', color)
            #     if blueprint.has_attribute('driver_id'):
            #         driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            #         blueprint.set_attribute('driver_id', driver_id)
            #     blueprint.set_attribute('role_name', 'autopilot')
            #     # spawn_point_v = waypoints[n].transform
            #     # prepare the light state of the cars to spawn
            #     light_state = vls.NONE
            #     if args.car_lights_on:
            #         light_state = vls.Position | vls.LowBeam | vls.LowBeam

            #     # spawn the cars and set their autopilot and light state all together
            #     batch.append(SpawnActor(blueprint, transform)
            #         .then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))
            #         .then(SetVehicleLightState(FutureActor, light_state)))

            # for response in client.apply_batch_sync(batch, synchronous_master):
            #     if response.error:
            #         logging.error(response.error)
            #         pass
            #     else:
            #         vehicles_list.append(response.actor_id)

            # -------------
            # Spawn Walkers
            # -------------
            # some settings
            percentagePedestriansRunning = 0.0      # how many pedestrians will run
            percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
            # 1. take all the random locations to spawn
            spawn_points = []
            for i in range(args.number_of_walkers):
                spawn_point = carla.Transform()
                loc = world.get_random_location_from_navigation()
                if (loc != None):
                    spawn_point.location = loc
                    spawn_points.append(spawn_point)
            # 2. we spawn the walker object
            batch = []
            walker_speed = []
            for spawn_point in spawn_points:
                walker_bp = random.choice(blueprintsWalkers)
                # set as not invincible
                if walker_bp.has_attribute('is_invincible'):
                    walker_bp.set_attribute('is_invincible', 'false')
                # set the max speed
                if walker_bp.has_attribute('speed'):
                    if (random.random() > percentagePedestriansRunning):
                        # walking
                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                    else:
                        # running
                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
                else:
                    print("Walker has no speed")
                    walker_speed.append(0.0)
                batch.append(SpawnActor(walker_bp, spawn_point))
            results = client.apply_batch_sync(batch, True)
            walker_speed2 = []
            for i in range(len(results)):
                if results[i].error:
                    # logging.error(results[i].error)
                    pass
                else:
                    walkers_list.append({"id": results[i].actor_id})
                    walker_speed2.append(walker_speed[i])
            walker_speed = walker_speed2
            # 3. we spawn the walker controller
            batch = []
            walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
            for i in range(len(walkers_list)):
                batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
            results = client.apply_batch_sync(batch, True)
            for i in range(len(results)):
                if results[i].error:
                    # logging.error(results[i].error)
                    pass
                else:
                    walkers_list[i]["con"] = results[i].actor_id
            # 4. we put altogether the walkers and controllers id to get the objects from their id
            for i in range(len(walkers_list)):
                all_id.append(walkers_list[i]["con"])
                all_id.append(walkers_list[i]["id"])
            all_actors = world.get_actors(all_id)

            # wait for a tick to ensure client receives the last transform of the walkers we have just created
            if not args.sync or not synchronous_master:
                world.wait_for_tick()
            else:
                world.tick()

            # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
            # set how many pedestrians can cross the road
            world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
            for i in range(0, len(all_id), 2):
                # start walker
                all_actors[i].start()
                # set walk to random point
                all_actors[i].go_to_location(world.get_random_location_from_navigation())
                # max speed
                all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

            print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

            # example of how to use parameters
            traffic_manager.global_percentage_speed_difference(30.0)

            while True:
                if args.sync and synchronous_master:
                    world.tick()
                else:
                    world.wait_for_tick()

        finally:

            if args.sync and synchronous_master:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)

            print('\ndestroying %d vehicles' % len(vehicles_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

            # stop walker controllers (list is [controller, actor, controller, actor ...])
            for i in range(0, len(all_id), 2):
                all_actors[i].stop()

            print('\ndestroying %d walkers' % len(walkers_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

            time.sleep(0.5)
    
    def setMap(self):
        print("Maps", self.client.get_available_maps())
        self.world = self.client.load_world('/Game/Carla/Maps/Town03') 

    def plotRoadid(self):
        # world_setting = carla.WorldSettings(synchronous_mode = True)
        # carla.World.apply_settings(world_setting)
        # self.client.reload_world()
        waypoints = self.client.get_world().get_map().generate_waypoints(distance=10.0)
        for i in range(len(waypoints)): # plot road id
                self.client.get_world().debug.draw_string(waypoints[i].transform.location, str(waypoints[i].road_id), draw_shadow=False,
                            color=carla.Color(r=0, g=0, b=0), life_time=100000,
                            persistent_lines=True)  

    # def camFaultParam(self):
    #     return camera_param

    # def imuFaultParam(self):
    #     return imu_param
    
    # def gpsFaultParam(self):
    #     return gps_param
    
    # def lidarFaultParam(self):
    #     return lidar_param
    
    # def faultID(self):
    #     return fault_id
    
    # def faultInjector(self): # RL
    #     # Deep RL Gen
    #     # -- Scenario param gen --
    #     # Distribute
    #     self.faultID() 
    #     return
    
    def run(self): # main loop
        r = rospy.Rate(10)
        # self.setMap()
        # while not rospy.is_shutdown():
        #     # self.setEgo()       
            
        #     if self.done == True:
        #         break
        #     r.sleep()
        self.setObstacle()
        # self.faultInjector()
        

# class FaultGenerationManager(object):

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    scenario_manager = ScenarioManager()
    sensor_fault = SensorFaultManager()
    try:
        scenario_manager.run()        
        # sensor_fault.run()
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
