#!/usr/bin/env python

import os
import resetworld
import time


def main():
    """
    main function
    """
    resetor = resetworld.ResetCarla()
    secs = time.time()
    try:
        print("Start FIT automation ...\n")
        os.system('./automation.sh')
        autoware_set_done = bool(input("Autoware set done? :: "))
        if autoware_set_done == True:
            print('Start Guardian... \n')
            os.system('./Guardian.sh')

        simulation_done = bool(input("Simulation is done? :: "))
        if simulation_done == True:
            print('Simulation is done... \n')
            print('Reset Carla world... \n')
            resetor.reset()
            print('Reset done \n')
            print('Kill Autoware ...\n')
            os.system('./automation_kill.sh')
            print("Finish")    
    finally:        
        pass


if __name__ == '__main__':
    try: 
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\n launch.')



