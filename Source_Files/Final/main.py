''' Description: This main python module invokes all scripts that are used 
in the Image Processing software system from capturing images of the field 
during the cruise out, target recognition, storing identifications and GPS
coordinates, mapping out a waypoint trajectory for dropping payloads at the
identification locations. The software system features involving stitching 
together images of the field to produce a digital map, and ultimately, overlaying
location pins and text boxes with GPS coordinates on the digital map will be 
performed manually using the command line interface after the flight.

'''
import math
import os
import time
import numpy as np
import dronekit
from dronekit import connect
from dronekit import VehicleMode
from pymavlink import mavutil

import run_mission as adpMiss
import calcGPS as GPS
import captureLiveVideoTest as CLVT 
import featRecog
import gen_published_map as gpm
#import png
import resize
import roll_collection as rc
import stitching
import undistort_frames as uf
import captureImages



def main():
    output = open('outputFile3.txt', 'w')
    # connection to pixhawk
    port = mavutil.auto_detect_serial(preferred_list=['*CubeOrange*if00'])
    print('connecting to vehicle', file=output)
    vehicle = connect(str(port[0]), wait_ready=True, baud=921600)
    
    alt = vehicle.location.global_relative_frame.alt
    print('alt is'+str(alt), file=output)
    runTime = 20 #sec
    maxRoll = 0.349066 #20 degrees in radians
    print('run mapGen', file=output)
    # Capture images of field during cruise out for stitching algorithm
    captureImages.mapImages(vehicle, output)

    # start condition: start camera when alt is greater than 150 ft(46m)
    t_end = time.time() + 60*5
    while True:
        if alt >= 46 or time.time() > t_end:
            print('inside loop, alt is: ' + str(alt), file=output) 
            lat, lon = CLVT.captureLiveVideoTest(vehicle, maxRoll, runTime, output)
            f = open('coordinates.txt', 'w')
            f.write(str(lat)+'\n'+str(lon)+'\n'+'success')
            vehicle.close()
            exit()
            # Must send coordinates of frowny face to ADP
            # We can add that as parameters to the method below
            # adpMiss.run_mission(lat, lon, vehicle)

        else:
            print("below", file=output)
            #time.sleep(1)
            alt = vehicle.location.global_relative_frame.alt
            print('alt is: '+ str(alt), file=output) 


if __name__ == "__main__":
    main()
