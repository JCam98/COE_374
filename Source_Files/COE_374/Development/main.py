<<<<<<< HEAD
import math
import time
import numpy as np
import dronekit
from dronekit import connect
from dronekit import VehicleMode

import run_mission as adpMiss
import calcGPS as GPS
import captureLiveVideoTest as CLVT 
import featRecog
import gen_published_map as gpm
import png
import resize
import roll_collection as rc
import stitching
import undistort_frames as uf



def main():
    # connection to pixhawk
    port = mavutil.auto_detect_serial(preferred_list=['*CubeOrange*if00'])
    vehicle = mavutil.mavlink_connection(str(port[0]), wait_ready=True, baud=921600)

    alt = vehicle.location.global_relative_frame.alt
    runTime = 60 #sec
    maxRoll = (math.pi)/2 #radians
    gps_coord = np.array([30.291466, -97.738195]) #hard code for ADP drop

    # start condition: start camera when alt is greater than 150 ft(46m)
    if alt >= 0:
        CLVT.captureLiveVideoTest(vehicle, maxRoll, runTime)
        adpMiss.run_mission(gps_coord, vehicle)

    else:
        time.sleep(1)
        alt = vehicle.location.global_relative_frame.alt


if __name__ == "__main__":
=======
import math
import time
import numpy as np
import dronekit
from dronekit import connect
from dronekit import VehicleMode

import run_mission as adpMiss
import calcGPS as GPS
import captureLiveVideoTest as CLVT 
import featRecog
import gen_published_map as gpm
import png
import resize
import roll_collection as rc
import stitching
import undistort_frames as uf



def main():
    # connection to pixhawk
    port = mavutil.auto_detect_serial(preferred_list=['*CubeOrange*if00'])
    vehicle = mavutil.mavlink_connection(str(port[0]), wait_ready=True, baud=921600)

    alt = vehicle.location.global_relative_frame.alt
    runTime = 60 #sec
    maxRoll = (math.pi)/2 #radians
    gps_coord = np.array([30.291466, -97.738195]) #hard code for ADP drop

    # start condition: start camera when alt is greater than 150 ft(46m)
    if alt >= 0:
        CLVT.captureLiveVideoTest(vehicle, maxRoll, runTime)
        adpMiss.run_mission(gps_coord, vehicle)

    else:
        time.sleep(1)
        alt = vehicle.location.global_relative_frame.alt


if __name__ == "__main__":
>>>>>>> 10c38ceeaa0b13ab1cc448b430fa2b4414b1806d
    main()