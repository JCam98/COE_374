import gen_published_map as gpm 
import numpy as np
import cv2
from PIL import Image, ImageFont, ImageDraw


textfile = "Final_Output/Refined_GPS_Coords.txt"
stitched_map = "final_undistorted_map.jpg"

gpm.gen_published_map(textfile, stitched_map)