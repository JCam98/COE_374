''' Property of: Justin Campbell
Description: This module contains the definition of a function "gen_published_map()"
that generates an image of the AOI with static text containing the GPS coordinates
and identifications of the targets and tarps. This function is imported into, 
and invoked in, "main.py". '''


import numpy as np
import cv2
from PIL import Image, ImageFont, ImageDraw


def gen_published_map(text_file_with_GPS_and_identifications, stitched_map_str):
    
    # Read text file into numpy array 
    
    GPS_idents_file_data = np.loadtxt(text_file_with_GPS_and_identifications, dtype = "str")
    
    # Extract GPS coordinates and identifications
    
    identifications = GPS_idents_file_data[:,0]
    GPS_lat_vec = GPS_idents_file_data[:,1]
    GPS_long_vec = GPS_idents_file_data[:,2]
    lat_range = [30.3230171, 30.325422] # Define boundaries of latitude in AOI
    long_range = [-97.6030071, -97.603750] # Define boundaries of longitude in AOI
    
    # Read in text box and location pins for overlay
    
    text_box = Image.open("text_box.jpg")
    loc_pin = Image.open("loc_pin.png")
    
    # Read in image of map from "stitching.py" into image object
    
    stitched_map = Image.open(stitched_map_str)
    stitched_map = stitched_map.rotate(180)
    stitched_map_2 = cv2.imread(stitched_map_str)
    
    # Return image size in pixels
    
    pixel_width_map, pixel_height_map, channel = stitched_map_2.shape
    
    # Loop through each GPS coordinate, for each identification, evaluate 
    # pixel location of centroid of target/tarp and overlay text box with 
    # strings containing GPS coordinate position and identification
    
    for i in range(0, len(identifications)):
        
        if (identifications[i] != 'none,'):
            
            # Convert latitude and longitude to floats
            
            GPS_lat = GPS_lat_vec[i][4:(len(GPS_lat_vec[i])-1)]
            GPS_lat = float(GPS_lat)
            GPS_long = GPS_long_vec[i][4:len(GPS_long_vec[i])]
            GPS_long = float(GPS_long)
            GPS_long_factor = (GPS_long - long_range[0]) / (long_range[1] - long_range[0])
            GPS_lat_factor = (GPS_lat - lat_range[0]) / (lat_range[1] - lat_range[0])
            
            pixel_x = round(GPS_long_factor * pixel_width_map)
            pixel_y = round(GPS_lat_factor * pixel_height_map)
            pixel_y = pixel_height_map - pixel_y # Account for origin at top left
            print(pixel_x)
            print(pixel_y)
            
            offset_x, offset_y = 100, 50 
            text_x = pixel_x + offset_x
            text_y = pixel_y + offset_y 
            
            text_color = (0,0,0) # Define RGB vector for black font text
            
            # Overlay location pin on stitched map
            
            stitched_map.paste(loc_pin, (pixel_x,pixel_y))
            
            # Overlay text box on stitched map
            # starting at coordinates (0, 0)
            
            stitched_map.paste(text_box, (text_x,text_y))
            
            title_text_ident = "Ident:" + identifications[i]
            image_editable = ImageDraw.Draw(stitched_map) # Image draw object
            image_editable.text((text_x, text_y), title_text_ident, text_color)
            
            # # Overlay GPS coordinate on map
            
            text_y = text_y + 10
            
            title_text_GPS_lat = "Lat: " + str(GPS_lat)
            title_text_GPS_long = "Long: " + str(GPS_long)
            image_editable = ImageDraw.Draw(stitched_map) # Image draw object
            image_editable.text((text_x, text_y), title_text_GPS_lat, text_color)
            text_y = text_y + 10
            image_editable.text((text_x, text_y), title_text_GPS_long, text_color)
                        
            
            stitched_map.save("Mission_Area_Map.png")        