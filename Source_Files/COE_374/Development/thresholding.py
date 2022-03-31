'''import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('it_100.png',0)
lower_bounds = (100,0,0)
upper_bounds = (225,80,80)
mask = cv2.inRange(img, lower_bounds, upper_bounds)
maskrgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
img = img & maskrgb
#ret,thresh1 = cv.threshold(img,200,255,cv.THRESH_BINARY)

	
cv2.imwrite('Test_gray.jpg', img) '''

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

ret,thresh1 = cv2.threshold(green,200,255,cv2.THRESH_BINARY)
gray = cv2.cvtColor(thresh1, cv2.COLOR_BGR2GRAY)

thresh = 20
im_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
cv2.imwrite('bw_image.png', im_bw)
# find and draw contours

contours = cv2.findContours(im_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
for contour in contours:
   cv2.drawContours(gray, contour, -1, (0, 255, 0), 3)
cv2.drawContours(im_bw, contours[0], -1, (0,255,0), 3)

plt.figure()
plt.imshow(im_bw)
plt.imsave("test_image.png",im_bw)
## save 
cv2.imwrite("green.png", thresh1)

