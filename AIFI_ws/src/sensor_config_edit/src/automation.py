#!/usr/bin/env python
import carla
from ast import In
import os
import resetworld
import time
import rosbag
import rospy
from autoware_msgs.msg import NDTStat
from std_msgs.msg import Int64

class automation(object):
    def __init__(self):
        self.status = False
        self.sim_done_status = False
        # self.client = carla.Client('192.168.0.121', '2000')

    def autowareSetCallback(self, data):
        checker = data.exe_time
        if checker != None:
            self.status = True
    
    def simDoneCallback(self, data):
        checker = data
        if checker != None:
            self.sim_done_status = True
        
    def listener(self):
        rospy.init_node("Simulator_checker")
        rospy.Subscriber("/ndt_stat", NDTStat, self.autowareSetCallback)
        rospy.Subscriber("/sim_done", Int64, self.simDoneCallback)

    def main(self):
        """
        main function
        """
        resetor = resetworld.ResetCarla()
        secs = time.time()    
        try:
            print("Start FIT automation ...\n")
            os.system('./automation.sh')
            self.listener()
            # autoware_set_done = bool(input("Autoware set done? :: "))
            while not rospy.is_shutdown():
                try:
                    if self.status == True:
                        print('Start Guardian... \n')
                        os.system('./Guardian.sh')
                        # os.system('~/bag.sh')
                        break
                    else:
                        rospy.loginfo_once('Waiting Autoware setting ...\n')
                except KeyboardInterrupt:
                    resetor.reset()
            # simulation_done = bool(input("Simulation is done? :: "))
            # simulation_done = 1
            if self.sim_done_status == True:
                print('Simulation is done... \n')
                print('Reset Carla world... \n')
                resetor.reset()
                # self.client.apply_batch([carla.command.DestroyActor()])
                print('Reset done \n')
                print('Kill Autoware ...\n')
                os.system('./automation_kill.sh')
                print("Finish")
            rospy.spin()
        finally:  
            # bag.close()      
            pass
        


if __name__ == '__main__':
    try:
        ai_fit = automation() 
        ai_fit.main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\n Done.')



