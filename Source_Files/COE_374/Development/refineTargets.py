'''Description: This module contains the definition of a function "captureLiveVideoTest()"
that continously captures live video and breaks the video into image frames
at hardcoded frame rate per second. For each frame that is output, the 
"undistort_frames()" function is invoked to remove tangential and radial distortion
from the images. Then, the resulting undistorted image frames are read into
"featRecog()" where SIFT and FLANN feature detection are performed to identify
images. The identifications, and their associated GPS coordinates are then 
written out to a text file. This function is imported into, 
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
            count_type_target[target['type']] += 1

        avg_lat = lat_sum/len(set_of_targets)
        avg_lon = lon_sum/len(set_of_targets)


        f.write(max(count_type_target, key=count_type_target.get)+'  lat='+str(avg_lat)+' , lon='+str(avg_lon)+'\n')