'''Description: This module contains the definition of a function "refineTargets()" which
activates the target refinement process. In particular, the array of targets/identifications
returned from the "findClosestTarget()" module are read in and all groupings where the total 
number of targets within the group is less than 4 are removed as false identifications. 
After the false identifications are removed, the average latitude and longitude for each grouping
of targets is evaluated along with the type of identification within that grouping. The 
target/identification type with the largest count in the group is then stored as the final designation
for that refined identification in a text file. This function marks the end of the refinement process
and the final classifications and GPS coordinates are written out to a text file for the 
Aerial Drop Payload Team to use to plan their waypoints for optimized payload delivery during
the next phase of the mission. This function is imported into, 
and invoked in, "main.py". '''


def refineTargets(target_arr, f):
    refined_targets = []
    for set_of_targets in target_arr:
        if len(set_of_targets) > 4:
            refined_targets.append(set_of_targets)
    for set_of_targets in refined_targets:
        lat_sum = 0
        lon_sum = 0
        count_type_target={'frowny':0, 'smiley':0, 'tarp':0}

        for target in set_of_targets:
            lat_sum += target['lat']
            lon_sum += target['lon']
            if target['type'] != 'none':
                count_type_target[target['type']] += 1

        avg_lat = lat_sum/len(set_of_targets)
        avg_lon = lon_sum/len(set_of_targets)


        f.write(max(count_type_target, key=count_type_target.get)+'  lat='+str(avg_lat)+' , lon='+str(avg_lon)+'\n')
