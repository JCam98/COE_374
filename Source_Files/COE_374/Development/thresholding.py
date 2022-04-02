import cv2
import numpy as np
import matplotlib.pyplot as plt

## Read
img = cv2.imread("it_100.png")

## convert to hsv
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

## mask of green (36,25,25) ~ (86, 255,255)
# mask = cv2.inRange(hsv, (36, 25, 25), (86, 255,255))
mask = cv2.inRange(hsv, (100, 25, 25), (125,255,255))

## slice the green
imask = mask>0
green = np.zeros_like(img, np.uint8)
green[imask] = img[imask]

cv2.imwrite("green.png", green)

ret,thresh1 = cv2.threshold(green,200,255,cv2.THRESH_BINARY)
gray = cv2.cvtColor(thresh1, cv2.COLOR_BGR2GRAY)

cv2.imwrite("gray.png", gray)

thresh = 20
im_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
cv2.imwrite('bw_image.png', im_bw)

# find and draw contours


contours = cv2.findContours(im_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

# Find the contour with the largest area

c = max(contours, key=cv2.contourArea)

# Find the bounding rectangle around the max contour (x,y) is top left coord, (w,h) is width and height

x,y,w,h = cv2.boundingRect(c)

# cv2.drawContours(img, c, -1, (0,255,0), 5)

cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

cv2.imwrite("outlined_image.png",img)

# Crop the image to the bounding rectangle only

crop_img = img[y:y+h, x:x+w]

cv2.imwrite("zoomed_in.png",crop_img)

cv2.imshow('Cropped Image', crop_img)
cv2.waitKey(0)

# FLANN Image Recognition
# img1 is the image with the features that we want to match to img2
img1 = cv2.imread("pos_image.png", cv2.IMREAD_GRAYSCALE)

img2 = cv2.imread("zoomed_in.png", cv2.IMREAD_GRAYSCALE)

#-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
minHessian = 400
detector = cv2.xfeatures2d.SURF_create(hessianThreshold=minHessian)
keypoints1, descriptors1 = detector.detectAndCompute(img1, None)
keypoints2, descriptors2 = detector.detectAndCompute(img2, None)
#-- Step 2: Matching descriptor vectors with a FLANN based matcher
# Since SURF is a floating-point descriptor NORM_L2 is used
matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
knn_matches = matcher.knnMatch(descriptors1, descriptors2, 2)
#-- Filter matches using the Lowe's ratio test
ratio_thresh = 0.7
good_matches = []
for m,n in knn_matches:
    if m.distance < ratio_thresh * n.distance:
        good_matches.append(m)
#-- Draw matches
img_matches = np.empty((max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1], 3), dtype=np.uint8)
cv2.drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
#-- Show detected matches
cv2.imshow('Good Matches', img_matches)
cv2.waitKey()
