import numpy as np
import cv2
import math
import time
import os
from os import listdir

import undistort_frames as uf

input = 'images/videoFrames'
output = 'images/image_dir'
count = 10001

for images in os.listdir(input):
    path = input + '/' + images
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)

    undst = uf.undistort_frames(img)

    cv2.imwrite(output +'/undst' +str(count)+'.jpg', undst)

    count += 1

print('Done.')
