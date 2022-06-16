'''Description: This module contains the definition of a function "captureLiveVideoTest()"
that continously captures live video and breaks the video into image frames
at hardcoded frame rate per second. For each frame that is output, the 
"undistort_frames()" function is invoked to remove tangential and radial distortion
from the images. Then, the resulting undistorted image frames are read into
"featRecog()" where SIFT and FLANN feature detection are performed to identify
images. The identifications, and their associated GPS coordinates are then 
written out to a text file. This function is imported into, 
and invoked in, "main.py". '''


import math

def findClosestTarget(candidate_target, target_arr):
    closest_dist = 10000
    closest_targ_spot = 0
    for spot in range(len(target_arr)):
        target_lat = target_arr[0]['lat']
        target_lon = target_arr[0]['lon']
        can_tar_lat = candidate_target['lat']
        can_tar_lon = candidate_target['lon']

        distAway = math.sqrt((target_lat - can_tar_lat)**2 + (target_lon - can_tar_lon)**2)

        if distAway <= closest_dist:
            closest_dist = distAway
            closest_targ_spot = spot
    
    return closest_dist, closest_targ_spot
