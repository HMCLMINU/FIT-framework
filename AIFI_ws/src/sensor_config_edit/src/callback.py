#!/usr/bin/env python

from tabnanny import check
import rospy
from autoware_msgs.msg import NDTStat
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64

sim_check = Int64()
sim_check = 0

def callback(data):
    
    checker = data.exe_time
    if checker != None:
        sim_check = 1
        pub.publish(sim_check)
    # rospy.loginfo("1")
    

rospy.init_node("Simulator_checker")
sub = rospy.Subscriber("/ndt_stat", NDTStat, callback)
pub = rospy.Publisher("/sim_checker", Int64, queue_size=1)


rospy.spin()