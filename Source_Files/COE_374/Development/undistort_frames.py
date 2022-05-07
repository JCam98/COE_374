''' Property of: Justin Campbell
Description: This module contains the definition of a function "undistort_frames()"
that uses a distance parameter and camera calibration matrix obtained from 
"camera_calibration_standard.py" to undistort image frames output from the 
"captureLiveVideoTest()" function. This function is imported into, 
and invoked in, "main.py". '''

import numpy as np
import cv2

def undistort_frames(frame):
    
    # Define Camera Calibration Parameters from "camera_calibration_standard.py"
        
    dist = np.array([[-0.279329,  -0.274769, -0.00572427, -0.0128338,  0.746644]])
    mtx = np.array([[587.394, 0, 329.668], [0, 590.266, 265.307], [0,0,1]])
    
    h,  w = frame.shape[:2] # Return frame size

    # Obtain new camera matrix for removing distortion from input image

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # # Undistortion Method: Using remapping

    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    frame = cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)
  
    #Crop the image
    x, y, w, h = roi
    frame = frame[y:y+h, x:x+w]

    return frame