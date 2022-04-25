import cv2
import os 
from os import listdir
import numpy as np
import matplotlib.pyplot as plt
import math
import sys

input = str(sys.argv[1])
output = str(sys.argv[2])
count = 10000

for images in os.listdir(input):
    img = cv2.imread(input + '/' + images, cv2.IMREAD_UNCHANGED)
    
    scale_percent = int(sys.argv[3]) # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)

    # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA) 
    cv2.imwrite(output +'/resized' +str(count)+'.jpg', resized)
    count += 1

print('Done. ' + str(count-10000) + " images resized to " + str(scale_percent) + "% of their original resolution.")
