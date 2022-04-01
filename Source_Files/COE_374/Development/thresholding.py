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

cv2.drawContours(img, contours, -1, (0,255,0), 5)

cv2.imwrite("outlined_image.png",img)
## save 
#cv2.imwrite("green.png", thresh1)
