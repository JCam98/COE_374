''' Property of: Target Acquired (Design Team 4)

    Purpose of Use: Raspberry Pi HQC Calibration
    
    Description: This python module is used to calibrate the camera lens of 
    the team's high quality camera to eliminate radial and tangential distortion.
    This is accomplished by defining intrinsic camera parameters (focal length, 
    optic center), as well as extrinsic parameters (geometry of sample object
    to map points from 3D world frame to 2D Image frame). Using an object (chessboard)
    with a well-defined pattern and geometry, the program will obtain a camera 
    calibration matrix that can ideally be used to transform any distorted image captured
    with the camera lens so that distorted curved lines are corrected to straight
    lines. This will be applicable to stitching together images for assembling 
    a 2D digital map of the field to ensure that there is no overlap between 
    adjacent images, and will likely be important for precisely calculating
    GPS coordinates of the centroid of area of identified targets. A chessboard with 
    9 x 9 point layout, and 7 x 7 internal grid points will be used for the calibration. 
    Each chess board grid square has a dimension of 47 centimeters and this dimension is used 
    to rescale the object points. '''                                                            


# Import modules

import numpy as np
import cv2 as cv
import glob
import os
from skimage import img_as_ubyte


# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Define interior (non-dimensionalized) object points:L(0,0,0), (1,0,0), (2,0,0) ....,(7,7,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
#objp = objp * 47 # Rescale each coordinate point according to 47 mm block width
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('/Users/jcam98/Desktop/EducationInstitutions/UniversityofTexas/Courses/SPRING_2022/COE_374/Design_Project/Locally_Developed_Files/Team_Chessboard_Images/*.jpg')

''' The following lines of code preceding the for loop are for testing only '''

image_test = "Team_Chessboard_Images/chess_center.jpg"
print (os.stat(image_test).st_size) # Output file size in bytes
img_array = []
img_array.append(image_test)
img = cv.imread(image_test)
window_name = "test"
cv.namedWindow(window_name)
#cv.startWindowThread()
cv.imshow(window_name, img)
cv.waitKey(5000)
cv.destroyWindow(window_name)
#cv.waitkey(1)

# print("OpenCV Build information")
# print(cv.getBuildInformation())

for fname in img_array:
    img = cv.imread(fname) # Read image file into "img" object
    #img = img *  1./5671599
    #img = cv.convertScaleAbs(img) # Convert image file size to 8 bits
    img_8bit = img_as_ubyte(img) # Convert image to 8 bit image
    # May need to change bit size of image to prevent information from being lost in conversion
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Map RGB colorspace to Grayscale colorspace
    window_name = "test" # Define window name for image display
    cv.namedWindow(window_name) # Pass window name into "cv" object
    #cv.startWindowThread()
    cv.imshow(window_name, gray) # Display grayscale image on window
    cv.waitKey(5000) # Pause execution for 5 seconds
    cv.destroyWindow(window_name) # Collapse window
    #cv.waitkey(1)
    #Find the chess board corners
    flags_findchessboard_corners = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FILTER_QUADS + cv.CALIB_CB_FAST_CHECK
    ret, corners = cv.findChessboardCorners(gray, (7,7), flags_findchessboard_corners)
    #flags_findchessboard_cornersSB = cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_ACCURACY + cv.CALIB_CB_LARGER + cv.CALIB_CB_MARKER
    #ret, corners = cv.findChessboardCornersSB(gray, (7,7), flags)
    #ret, corners = cv.findChessboardCornersSB(gray, (7,7), flag) #, cv.CALIB_CB_ACCURACY)
    #ret, corners = cv.findChessboardCornersSB(gray, (7,7), None)
    print(ret) #testing only
    print(corners) # testing only
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,7), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(5000)
cv.destroyWindow(window_name)


# ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# img = cv.imread('left12.jpg')
# h,  w = img.shape[:2]
# newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# # undistort
# dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# # crop the image
# x, y, w, h = roi
# dst = dst[y:y+h, x:x+w]
# cv.imwrite('calibresult.png', dst)

# mean_error = 0
# for i in range(len(objpoints)):
#     imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
#     error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
#     mean_error += error
# print( "total error: {}".format(mean_error/len(objpoints)) )
