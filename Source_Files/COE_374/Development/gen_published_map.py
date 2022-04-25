''' Property of: Justin Campbell
Description: This module contains the definition of a function "gen_published_map()"
that generates an image of the AOI with static text containing the GPS coordinates
and identifications of the targets and tarps. This function is imported into, 
and invoked in, "main.py". '''


import numpy as np
import cv2
from PIL import Image, ImageFont, ImageDraw



def gen_published_map(text_file_with_GPS_and_identifications, stitched_map):
    
    # Read text file into numpy array 
    
    GPS_idents_file_data = np.loadtxt(text_file_with_GPS_and_identifications, dtype = str)
    
    # Extract GPS coordinates and identifications
    
    GPS_coords = GPS_idents_file_data[:,0]
    identifications = GPS_idents_file_data[:,1]
    
    # Read in image of map from "stitching.py" into image object
    
    stitched_map = cv2.imread(stitched_map)
    
    # Loop through each GPS coordinate, for each identification, evaluate 
    # pixel location of centroid of target/tarp and overlay text box with 
    # strings containing GPS coordinate position and identification
    
    for i in range(0, len(GPS_coords)):
        
        if (identifications[i] != "None"):
            
            # Calculate pixel location of centroid of target/tarp and overlay text
            # box with strings containing GPS coordinate position and identification
            # using "calcGPS()"
            
            #pixel_x = (xChange/(mPerPixel*convertDeg)) + lon_center
            #pixel_y = (yChange/(mPerPixel*convertDeg)) + lat_center
            #xChange = (xpoint - lon_center)*mPerPixel*convertDeg
           # yChange = (ypoint - lat_center)*mPerPixel*convertDeg

            
            # Offset pixel location of beginning of overlayed text 
            
            offset_x, offset_y = 100, 50 
            text_x = pixel_x + offset_x
            text_y = pixel_y + offset_y 
            
            text_color = (237, 230, 211) 
            
            # Overlay identification on map
            
            title_text_ident = "Identification: " + identifications[i] 
            image_editable = ImageDraw.Draw(stitched_map) # Image draw object
            image_editable.text((text_x, text_y), title_text_ident, text_color)
            
            # Overlay GPS coordinate on map
            
            text_y = text_y + 50
            
            title_text_GPS = "GPS Coordinate: " + GPS_coords[i]
            image_editable = ImageDraw.Draw(stitched_map) # Image draw object
            image_editable.text((text_x, text_y), title_text_GPS, text_color)
            
    stitched_map.save("Mission_Area_Map.png")        