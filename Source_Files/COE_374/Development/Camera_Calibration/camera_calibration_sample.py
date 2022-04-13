''' Property of: Target Acquired (Design Team 4)

    Purpose of Use: Raspberry Pi HQC Calibration
    
    Description: This script is a replication of the camera calibration algorithm presented in the 
    OpenCV tutorial and used to guide and refine the results of the algorithm in 
    "camera_calibration.py". '''                                                            


# Import modules

import numpy as np
import cv2 as cv
import glob
import os


# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Define interior (non-dimensionalized) object points:L(0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('/Users/jcam98/Desktop/EducationInstitutions/UniversityofTexas/Courses/SPRING_2022/COE_374/Design_Project/Locally_Developed_Files/Sample_Repo_Images/*.jpg')

for fname in images:
    img = cv.imread(fname) # Read image file into "img" object
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Map RGB colorspace to Grayscale colorspace
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(5000)
cv.destroyAllWindows()
print(ret)
print(corners)

# Return camera matrix, distortion coefficients, rotation and translation vectors

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

img = cv.imread('Sample_Repo_Images/left12.jpg')
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('Sample_Repo_Images/calibresult.png', dst)
print (os.stat('Sample_Repo_Images/left12.jpg').st_size) # Output file size in bytes

# mean_error = 0
# for i in range(len(objpoints)):
#     imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
#     error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
#     mean_error += error
# print( "total error: {}".format(mean_error/len(objpoints)) )
