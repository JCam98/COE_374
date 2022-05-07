'''Description: This module contains the definition of a function "captureLiveVideoTest()"
that continously captures live video and breaks the video into image frames
at hardcoded frame rate per second. For each frame that is output, the 
"undistort_frames()" function is invoked to remove tangential and radial distortion
from the images. Then, the resulting undistorted image frames are read into
"featRecog()" where SIFT and FLANN feature detection are performed to identify
images. The identifications, and their associated GPS coordinates are then 
written out to a text file. This function is imported into, 
and invoked in, "main.py". '''

from featRecog import featRecog 
import undistort_frames as uf
import refineTargets as rt
import findClosestTarget as fct
import time
import cv2


def captureLiveVideoTest(vehicle, maxRoll, runTime, output):
    print('inside videoCapture')
    print('inside livevideotest', file=output)
    cap = cv2.VideoCapture(0)
    cap.set(3,1920)
    cap.set(4,1080)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    count = 0
    target_list = []
    g = open('Final_Output/GPS_coords.txt'+str(time.time_ns()), 'a')
    f = open('Final_Output/Refined_GPS_Coords.txt'+str(time.time_ns()), 'a')
 

    # Check next waypoint
    nextWay = vehicle.commands.next
    print('next waypoint:' + str(nextWay), file=output)
        
    # While haven't reached last waypoint in search, keep searching
    while nextWay < 29:
        print('inside loop', file=output)
        nextWay = vehicle.commands.next
        print('next waypoint:' + str(nextWay), file=output)
        # Capture frame-by-frame
        ret, frame = cap.read()
        # Undisort image frame
        #frame = uf.undistort_frames(frame)
        # pull GPS
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        alt = vehicle.location.global_relative_frame.alt

        print('alt is: ' + str(alt), file=output)
        print('lat is: ' + str(lat), file=output)
        print('lon is: ' + str(lon), file=output)

        #test
        #lat = vehicle.location.LocationGlobalRelative.lat
        #lon = vehicle.location.LocationGlobalRelative.long
        #alt = vehicle.location.LocationGlobalRelative.alt 
        #print("lat: " + str(lat))
        #print("long: " + str(long))
        #print("alt: " + str(alt))

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        print('about to run featRecog', file=output)
        target_type, target_lat, target_lon  = featRecog(frame, count, lat, lon, alt)
        print('feat recog has run', file=output)
        if target_type == 'frowny':
            frownyLat = target_lat
            frownyLon = target_lon
        g.write(target_type+'  lat='+str(target_lat)+' , lon='+str(target_lon)+'\n')
        candidate_target = {'type':target_type, 'lat':target_lat, 'lon':target_lon}
        '''
        if not target_list:
            target_list.append(candidate_target)
            print(target_list)
        else:
            print(candidate_target)
            print(target_list)
            dist, spot_in_list = fct.findClosestTarget(candidate_target, target_list)
            if dist < .0001:
                print(target_list)
                print(type(target_list))
                target_list[spot_in_list].append(candidate_target)
            else:
                target_list[-1] = [candidate_target]
            print(target_list)
        '''
        print('increment count', file=output)
        count += 1
    
    #rt.refineTargets(target_list, f)
    

    # When everything done, release the capture
    print('Done, release capture', file=output)
    cap.release()
    cv2.destroyAllWindows()
        
    f.close()
    return target_lat, target_lon
