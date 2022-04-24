import math
import time
import numpy as np
import dronekit
from dronekit import connect
from dronekit import VehicleMode

import run_mission as adpMiss
import conOps_functions as atr

def main():
    # connection to pixhawk
    port = mavutil.auto_detect_serial(preferred_list=['*CubeOrange*if00'])
    vehicle = mavutil.mavlink_connection(str(port), baud=921600)

    alt = vehicle.location.global_relative_frame.alt
    runTime = 60 #sec
    maxRoll = (math.pi)/2 #radians
    gps_coord = np.array([30.291466, -97.738195]) #hard code for ADP drop

    # start condition: start camera when alt is greater than 150 ft(46m)
    if alt >= 0:
        atr.captureLiveVideoTest(vehicle, maxRoll, runTime)
        adpMiss.run_mission(gps_coord, vehicle)

    else:
        time.sleep(1)
        alt = vehicle.location.global_relative_frame.alt


if __name__ == "__main__":
    main()