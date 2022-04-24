''' Property of: Target Acquired (Design Team 4)

    Purpose of Use: Raspberry Pi HQC Calibration
    
    Description: This script is an application of the camera calibration algorithm presented in the 
    OpenCV tutorial and uses distorted images of a chessboard to obtain intrinsic 
    properties of a camera such as focal length and optic center. Ultimately, 
    these properties generate two variables, "dist", and a camera calibration 
    matrix, "mtx" that will be hardcoded in the "thresholding.py" script used
    for target recognition and the "stitching.py" script used for map generation
    to undistort images of the field. This will theoretically improve the accuracy
    of image identification, GPS calculations, and map stitching. '''                                                          


# Import modules

import numpy as np
import cv2 as cv
import glob
import os

os.chdir("/Users/jcam98/Desktop/EducationInstitutions/UniversityofTexas/Courses/SPRING_2022/COE_374/Design_Project/Locally_Developed_Files/Camera_Calibration/Test_Images/trial_5")

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Define interior (non-dimensionalized) object points:L(0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((8*6,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:8].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('/Users/jcam98/Desktop/EducationInstitutions/UniversityofTexas/Courses/SPRING_2022/COE_374/Design_Project/Locally_Developed_Files/Camera_Calibration/Team_Chessboard_Images/Black_White/It_4/*.jpg')

for fname in images:
    img = cv.imread(fname) # Read image file into "img" object
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Map RGB colorspace to Grayscale colorspace
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (6,8), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (6,8), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(5000)
cv.destroyAllWindows()

# Return camera matrix, distortion coefficients, rotation and translation vectors

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


images = sorted(glob.glob('*.jpg'), key=os.path.basename)


count = 1
for i in images: 

    i = cv.imread(i)

    h,  w = i.shape[:2]
    
    #h1, w1 = 5 * h, 5 * w
    
    # # Obtain new camera matrix for removing distortion from input image
    
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    
    # # Undisortion Method 1: cv.undistort() 
    
    #dst = cv.undistort(i, mtx, dist, None, newcameramtx)
    
    # # Undistortion Method 2: Using remapping
    
    mapx,mapy = cv.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    dst = cv.remap(i,mapx,mapy,cv.INTER_LINEAR)
  
    #crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    
    out_file = 'undst/image' + str(count) + '_undst.jpg'
    
    cv.imwrite(out_file, dst)
    
    count = count + 1
    
    # # mean_error = 0
    # for i in range(len(objpoints)):
    #     imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    #     error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    #     mean_error += error
    # print( "total error: {}".format(mean_error/len(objpoints)) )
