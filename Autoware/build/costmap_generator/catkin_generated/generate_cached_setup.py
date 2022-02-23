# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/autoware/Autoware/install/object_map;/home/autoware/Autoware/install/lanelet2_extension;/home/autoware/Autoware/install/vector_map;/home/autoware/Autoware/install/amathutils_lib;/home/autoware/Autoware/install/vector_map_msgs;/home/autoware/Autoware/install/autoware_msgs;/home/autoware/Autoware/install/autoware_lanelet2_msgs;/home/autoware/Autoware/install/autoware_build_flags;/home/autoware/Autoware/install/ymc;/home/autoware/Autoware/install/xsens_driver;/home/autoware/Autoware/install/wf_simulator;/home/autoware/Autoware/install/lattice_planner;/home/autoware/Autoware/install/waypoint_planner;/home/autoware/Autoware/install/waypoint_maker;/home/autoware/Autoware/install/way_planner;/home/autoware/Autoware/install/vlg22c_cam;/home/autoware/Autoware/install/vision_ssd_detect;/home/autoware/Autoware/install/vision_segment_enet_detect;/home/autoware/Autoware/install/vision_lane_detect;/home/autoware/Autoware/install/vision_darknet_detect;/home/autoware/Autoware/install/vision_beyond_track;/home/autoware/Autoware/install/vel_pose_diff_checker;/home/autoware/Autoware/install/vehicle_socket;/home/autoware/Autoware/install/vehicle_sim_model;/home/autoware/Autoware/install/vehicle_model;/home/autoware/Autoware/install/vehicle_gazebo_simulation_launcher;/home/autoware/Autoware/install/vehicle_gazebo_simulation_interface;/home/autoware/Autoware/install/vehicle_engage_panel;/home/autoware/Autoware/install/vehicle_description;/home/autoware/Autoware/install/trafficlight_recognizer;/home/autoware/Autoware/install/op_utilities;/home/autoware/Autoware/install/op_simulation_package;/home/autoware/Autoware/install/op_local_planner;/home/autoware/Autoware/install/op_global_planner;/home/autoware/Autoware/install/lidar_kf_contour_track;/home/autoware/Autoware/install/op_ros_helpers;/home/autoware/Autoware/install/ff_waypoint_follower;/home/autoware/Autoware/install/dp_planner;/home/autoware/Autoware/install/op_simu;/home/autoware/Autoware/install/op_planner;/home/autoware/Autoware/install/op_utility;/home/autoware/Autoware/install/lidar_euclidean_cluster_detect;/home/autoware/Autoware/install/vector_map_server;/home/autoware/Autoware/install/road_occupancy_processor;/home/autoware/Autoware/install/costmap_generator;/home/autoware/Autoware/install/naive_motion_predict;/home/autoware/Autoware/install/lanelet_aisan_converter;/home/autoware/Autoware/install/map_file;/home/autoware/Autoware/install/libvectormap;/home/autoware/Autoware/install/lane_planner;/home/autoware/Autoware/install/imm_ukf_pda_track;/home/autoware/Autoware/install/decision_maker;/home/autoware/Autoware/install/vectacam;/home/autoware/Autoware/install/udon_socket;/home/autoware/Autoware/install/twist_generator;/home/autoware/Autoware/install/twist_gate;/home/autoware/Autoware/install/twist_filter;/home/autoware/Autoware/install/twist2odom;/home/autoware/Autoware/install/tablet_socket;/home/autoware/Autoware/install/runtime_manager;/home/autoware/Autoware/install/mqtt_socket;/home/autoware/Autoware/install/tablet_socket_msgs;/home/autoware/Autoware/install/state_machine_lib;/home/autoware/Autoware/install/sound_player;/home/autoware/Autoware/install/sick_lms5xx;/home/autoware/Autoware/install/sick_ldmrs_tools;/home/autoware/Autoware/install/sick_ldmrs_driver;/home/autoware/Autoware/install/sick_ldmrs_msgs;/home/autoware/Autoware/install/sick_ldmrs_description;/home/autoware/Autoware/install/points2image;/home/autoware/Autoware/install/rosinterface;/home/autoware/Autoware/install/rosbag_controller;/home/autoware/Autoware/install/pure_pursuit;/home/autoware/Autoware/install/points_preprocessor;/home/autoware/Autoware/install/mpc_follower;/home/autoware/Autoware/install/lidar_localizer;/home/autoware/Autoware/install/emergency_handler;/home/autoware/Autoware/install/autoware_health_checker;/home/autoware/Autoware/install/as;/home/autoware/Autoware/install/ros_observer;/home/autoware/Autoware/install/roi_object_filter;/home/autoware/Autoware/install/range_vision_fusion;/home/autoware/Autoware/install/pos_db;/home/autoware/Autoware/install/points_downsampler;/home/autoware/Autoware/install/pixel_cloud_fusion;/home/autoware/Autoware/install/pcl_omp_registration;/home/autoware/Autoware/install/pc2_downsampler;/home/autoware/Autoware/install/oculus_socket;/home/autoware/Autoware/install/obj_db;/home/autoware/Autoware/install/nmea_navsat;/home/autoware/Autoware/install/ndt_tku;/home/autoware/Autoware/install/ndt_gpu;/home/autoware/Autoware/install/ndt_cpu;/home/autoware/Autoware/install/multi_lidar_calibrator;/home/autoware/Autoware/install/microstrain_driver;/home/autoware/Autoware/install/memsic_imu;/home/autoware/Autoware/install/marker_downsampler;/home/autoware/Autoware/install/map_tools;/home/autoware/Autoware/install/map_tf_generator;/home/autoware/Autoware/install/log_tools;/home/autoware/Autoware/install/lidar_shape_estimation;/home/autoware/Autoware/install/lidar_point_pillars;/home/autoware/Autoware/install/lidar_naive_l_shape_detect;/home/autoware/Autoware/install/lidar_fake_perception;/home/autoware/Autoware/install/lidar_apollo_cnn_seg_detect;/home/autoware/Autoware/install/libwaypoint_follower;/home/autoware/Autoware/install/lgsvl_simulator_bridge;/home/autoware/Autoware/install/kvaser;/home/autoware/Autoware/install/kitti_launch;/home/autoware/Autoware/install/kitti_player;/home/autoware/Autoware/install/kitti_box_publisher;/home/autoware/Autoware/install/javad_navsat_driver;/home/autoware/Autoware/install/integrated_viewer;/home/autoware/Autoware/install/image_processor;/home/autoware/Autoware/install/hokuyo;/home/autoware/Autoware/install/graph_tools;/home/autoware/Autoware/install/gnss_localizer;/home/autoware/Autoware/install/gnss;/home/autoware/Autoware/install/glviewer;/home/autoware/Autoware/install/gazebo_world_description;/home/autoware/Autoware/install/gazebo_imu_description;/home/autoware/Autoware/install/gazebo_camera_description;/home/autoware/Autoware/install/garmin;/home/autoware/Autoware/install/freespace_planner;/home/autoware/Autoware/install/fastvirtualscan;/home/autoware/Autoware/install/ekf_localizer;/home/autoware/Autoware/install/ds4_msgs;/home/autoware/Autoware/install/ds4_driver;/home/autoware/Autoware/install/detected_objects_visualizer;/home/autoware/Autoware/install/decision_maker_panel;/home/autoware/Autoware/install/data_preprocessor;/home/autoware/Autoware/install/custom_msgs;/home/autoware/Autoware/install/carla_autoware_bridge;/home/autoware/Autoware/install/calibration_publisher;/home/autoware/Autoware/install/autoware_system_msgs;/home/autoware/Autoware/install/autoware_rviz_plugins;/home/autoware/Autoware/install/autoware_quickstart_examples;/home/autoware/Autoware/install/autoware_pointgrey_drivers;/home/autoware/Autoware/install/autoware_driveworks_interface;/home/autoware/Autoware/install/autoware_connector;/home/autoware/Autoware/install/autoware_camera_lidar_calibrator;/home/autoware/Autoware/install/astar_search;/home/autoware/Autoware/install/autoware_map_msgs;/home/autoware/Autoware/install/autoware_launcher_rviz;/home/autoware/Autoware/install/autoware_launcher;/home/autoware/Autoware/install/autoware_external_msgs;/home/autoware/Autoware/install/autoware_driveworks_gmsl_interface;/home/autoware/Autoware/install/autoware_config_msgs;/home/autoware/Autoware/install/autoware_can_msgs;/home/autoware/Autoware/install/autoware_bag_tools;/home/autoware/Autoware/install/adi_driver;/home/autoware/carla_ws/devel;/opt/ros/melodic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/autoware/Autoware/build/costmap_generator/devel/env.sh')

output_filename = '/home/autoware/Autoware/build/costmap_generator/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
