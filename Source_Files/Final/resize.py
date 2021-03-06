''' Property of: Justin Campbell
Description: This module contains the definition of a function "resize()"
that reduces the resolution of the images captured in "captureliveVideoTest()" to optimize
the runtime and accuracy of the image stitching algorithm in "stitching.py" that
generates a digital map of the AOI. This function is imported into, 
and invoked in, "main.py". '''

import cv2
import os 
from os import listdir
import numpy as np
import matplotlib.pyplot as plt
import math
import sys

def resize(frame, count):

    count = count + 10001
    scale_percent = 75 # percent of original size
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)

    # resize image
    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA) 
    cv2.imwrite('images/resized/resized' + str(count) + '.jpg', frame)

    return frame
