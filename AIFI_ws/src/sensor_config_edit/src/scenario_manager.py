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


class ScenarioManager(object):
    def __init__(self):
        rospy.init_node('scenario_Manager', anonymous=True)
        self.client = carla.Client('192.168.0.121', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.startPosePub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.goalPosePub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.done = False

    def setEgo(self): # set ego state publish

        filtered_waypoints = self.egoselectWPT(self.world) 
        # set SV spawn point selection  
        spawn_point_v = filtered_waypoints[38].transform
        # set a destination of SV
        destination = filtered_waypoints[1].transform
        spawn_quat = quaternion_from_euler(math.radians(spawn_point_v.rotation.roll),
        math.radians(spawn_point_v.rotation.pitch), math.radians(spawn_point_v.rotation.yaw))
        dest_quat = quaternion_from_euler(math.radians(destination.rotation.roll),
        math.radians(destination.rotation.pitch), math.radians(destination.rotation.yaw))

        startpose = PoseWithCovarianceStamped()
        goalpose = PoseStamped()
        now = rospy.get_rostime()
        
        startpose.header.frame_id = "map"
        startpose.header.stamp = now
        startpose.pose.pose.position.x = spawn_point_v.location.x
        startpose.pose.pose.position.y = -spawn_point_v.location.y
        startpose.pose.pose.position.z = 0
        startpose.pose.pose.orientation.x = spawn_quat[0]
        startpose.pose.pose.orientation.y = spawn_quat[1]
        startpose.pose.pose.orientation.z = spawn_quat[2]
        startpose.pose.pose.orientation.w = spawn_quat[3]
        startpose.pose.covariance = np.zeros(36)

        goalpose.header.frame_id = "map"
        goalpose.header.stamp = rospy.Time.now()
        goalpose.pose.position.x = destination.location.x
        goalpose.pose.position.y = -destination.location.y
        goalpose.pose.position.z = 0
        goalpose.pose.orientation.x = dest_quat[0]
        goalpose.pose.orientation.y = dest_quat[1]
        goalpose.pose.orientation.z = dest_quat[2]
        goalpose.pose.orientation.w = dest_quat[3]

        if now < rospy.Time(1):
            pass
        else:
            # print("now : {}".format(now))
            self.startPosePub.publish(startpose)
            self.goalPosePub.publish(goalpose)
            



    def setObstacle(self): # set obstacle state
        # return start_position, goal_position, start_vel

        args = easydict.EasyDict({
            "host" : '192.168.0.121',
            "port" : 2000,
            "number_of_vehicles" : 0,
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
        done = 0
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

            # plot road id
            self.plotRoadid()
            # waypoints selection
            filtered_waypoints = self.selectWPT(world) 
            vehicle_blueprint = client.get_world().get_blueprint_library().filter('model3')[0] # vehicle object
            # set SV spawn point selection  
            spawn_point_v = filtered_waypoints[7].transform
            spawn_point_v.location.z += 2
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
                    self.reset()
                    print("The target has been reached, stopping the simulation")
                    break
                vehicle.apply_control(agent.run_step())
                if done == 0:
                    print("Control surrounding vehicles ...")
                    done = 1

            
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

    def selectWPT(self, world):
        ##### Filter road #####
        # get spawn points from map
        spawn_points = world.get_map().get_spawn_points()
        # get spawn points from map by 5 meter
        waypoints = self.client.get_world().get_map().generate_waypoints(distance=5.0)
        filtered_waypoints = []
        for waypoint in waypoints:
            if(waypoint.lane_id == 1) and (waypoint.road_id == 1 or waypoint.road_id == 25): # cluster by road_id and lane_id
                filtered_waypoints.append(waypoint) # add waypoints

        for i in range(len(filtered_waypoints)): # plot added waypoints
            self.client.get_world().debug.draw_string(filtered_waypoints[i].transform.location, str(i), draw_shadow=False,
                        color=carla.Color(r=0, g=255, b=0), life_time=100000,
                        persistent_lines=True) 
        return filtered_waypoints

    def egoselectWPT(self, world):
        ##### Filter road #####
        # get spawn points from map
        spawn_points = world.get_map().get_spawn_points()
        # get spawn points from map by 5 meter
        waypoints = self.client.get_world().get_map().generate_waypoints(distance=5.0)
        filtered_waypoints = []
        for waypoint in waypoints:
            if(waypoint.lane_id == 1) and (waypoint.road_id == 1 or waypoint.road_id == 2): # cluster by road_id and lane_id
                filtered_waypoints.append(waypoint) # add waypoints

        for i in range(len(filtered_waypoints)): # plot added waypoints
            self.client.get_world().debug.draw_string(filtered_waypoints[i].transform.location, str(i), draw_shadow=False,
                        color=carla.Color(r=0, g=0, b=255), life_time=100000,
                        persistent_lines=True) 
        return filtered_waypoints

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

    def reset(self):
        self.client.reload_world()

    def run(self): # main loop
        r = rospy.Rate(10)
        j = 0
        # self.setMap()
        while not rospy.is_shutdown():
            self.setEgo()
            if j > 1:
                self.done=True

            if self.done == True:
                break
            j += 1
            r.sleep()
       
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
    try:
        scenario_manager.run()        
    finally:
        if scenario_manager is not None:
            # scenario_manager.destroy()
            pass


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
