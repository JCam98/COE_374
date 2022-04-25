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
        
    dist = np.array([[-0.50674759,  0.23775016, -0.01112213, -0.00096096,  0.17855823]])
    mtx = np.array([[1882.35, 0, 959.767], [0, 1875.87, 575.389]])
    
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