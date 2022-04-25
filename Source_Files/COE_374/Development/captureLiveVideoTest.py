'''Description: This module contains the definition of a function "captureLiveVideoTest()"
that continously captures live video and breaks the video into image frames
at hardcoded frame rate per second. For each frame that is output, the 
"undistort_frames()" function is invoked to remove tangential and radial distortion
from the images. Then, the resulting undistorted image frames are read into
"featRecog()" where SIFT and FLANN feature detection are performed to identify
images. The identifications, and their associated GPS coordinates are then 
written out to a text file. This function is imported into, 
and invoked in, "main.py". '''

import featRecog 
import undistort_frames as uf
import time
import cv2


def captureLiveVideoTest():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    count = 0
    f = open('GPS_coords.txt'+str(time.time_ns()), 'a')
    while cap.isOpened():
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Undisort image frame 
        
        frame = uf(frame)
        
        # pull GPS
#        lat = vehicle.location.global_relative_frame.lat
#        lon = vehicle.location.global_relative_frame.lon
#        alt = vehicle.location.global_relative_frame.alt

        lat = 30.32420
        lon = -97.60313
        alt = 76.2
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        target_type, target_lat, target_lon  = featRecog(frame, count, lat, lon, alt)
        f.write(target_type+'  lat='+str(target_lat)+' , lon='+str(target_lon)+'\n')
        # Display the resulting frame
        #cv2.imwrite('frame' +str(count)+'.png', gray)
        count += 1


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
        
    f.close()