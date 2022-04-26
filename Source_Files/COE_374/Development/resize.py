import cv2
import os 
from os import listdir
import numpy as np
import matplotlib.pyplot as plt
import math
import sys

def resize(frame, count):

    count = count + 10001
    scale_percent = 66 # percent of original size
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)

    # resize image
    resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA) 
    cv2.imwrite('Test_Images/trial_5/resize' +str(count)+'.jpg', resized)

    return frame