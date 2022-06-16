'''Description: This module contains the definition of a function "findClosestTarget()" which 
groups identifications together that are within close proximity to each other
 This function is imported into, and invoked in, "main.py". '''


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
