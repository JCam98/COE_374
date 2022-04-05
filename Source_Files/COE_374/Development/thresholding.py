import cv2
import numpy as np
import matplotlib.pyplot as plt


## Read
img = cv2.imread("inputImages/it_1.png")

## convert to hsv
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

## mask of green (36,25,25) ~ (86, 255,255)
# mask = cv2.inRange(hsv, (36, 25, 25), (86, 255,255))
mask = cv2.inRange(hsv, (100, 25, 25), (125,255,255))

## slice the green
imask = mask>0
green = np.zeros_like(img, np.uint8)
green[imask] = img[imask]

#cv2.imwrite("green.png", green)

ret,thresh1 = cv2.threshold(green,200,255,cv2.THRESH_BINARY)
gray = cv2.cvtColor(thresh1, cv2.COLOR_BGR2GRAY)

#cv2.imwrite("gray.png", gray)

thresh = 20
im_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
cv2.imwrite('testImages/bw_image.png', im_bw)
# find and draw contours

contours = cv2.findContours(im_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

#cv2.drawContours(img, contours, -1, (0,255,0), 5)
#cv2.imwrite('testImages/contour_image.png',img)

# Find the contour with the largest area

c = max(contours, key=cv2.contourArea)

# Find the bounding rectangle around the max contour (x,y) is top left coord, (w,h) is width and height

x,y,w,h = cv2.boundingRect(c)

#cv2.drawContours(img, c, -1, (0,255,0), 5)

#cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

cv2.imwrite("outputImages/outlined_image.png",img)

# Crop the image to the bounding rectangle only

crop_img = img[y:y+h, x:x+w]
crop_bw = im_bw[y:y+h, x:x+w]

cv2.imwrite("outputImages/zoomed_in.png",crop_img)
cv2.imwrite("outputImages/zoomed_inbw.png", crop_bw)

im = cv2.imread('outputImages/zoomed_inbw.png')
invert = cv2.bitwise_not(im)
imgray = cv2.cvtColor(invert, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(imgray, 127, 255, 0)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

c = max(contours, key=cv2.contourArea)

#(x,y),radius = cv2.minEnclosingCircle(c)
#center = (int(x),int(y))
#radius = int(radius)

#cv2.circle(crop_img,center,radius,(0,255,0),2)

x,y,w,h = cv2.boundingRect(c)
face = crop_img[y:y+h, x:x+w]

cv2.imwrite("outputImages/face.png",face)

#cv2.drawContours(invert, c, -1, (0,255,0), 5)

#cv2.imshow('Cropped Image', crop_img)
#cv2.waitKey(0)

# SIFT Feature Matching

img1 = cv2.imread('inputImages/queryImageSmiley.png',cv2.IMREAD_GRAYSCALE) # queryImage
img2 = cv2.imread('outputImages/face.png',cv2.IMREAD_GRAYSCALE) # trainImage

# Initiate SIFT detector
sift = cv2.SIFT_create()

# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

# BFMatcher with default params
bf = cv2.BFMatcher()
matches = bf.knnMatch(des1,des2,k=2)

# Apply ratio test
good = []
for m,n in matches:
    if m.distance < 0.75*n.distance:
        good.append([m])

# cv.drawMatchesKnn expects list of lists as matches.
img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
plt.imshow(img3),plt.show()

cv2.imwrite("outputImages/SIFT_test.png",img3)