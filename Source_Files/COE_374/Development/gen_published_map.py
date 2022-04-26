''' Property of: Justin Campbell
Description: This module contains the definition of a function "gen_published_map()"
that generates an image of the AOI with static text containing the GPS coordinates
and identifications of the targets and tarps. This function is imported into, 
and invoked in, "main.py". '''


import numpy as np
from PIL import Image, ImageDraw



def gen_published_map(textfile, stitched_map, xpoint_vec, ypoint_vec):
    
    # Read text file into numpy array 
    
    GPS_idents_file_data = np.loadtxt(textfile, dtype = str)
    
    # Extract GPS coordinates and identifications
    
    identifications = GPS_idents_file_data[:,0]
    GPS_coords_lat = GPS_idents_file_data[:,1]
    GPS_coords_long = GPS_idents_file_data[:,2]
    GPS_coords = []
    lat_range = [bottom, top] # Define boundaries of latitude in AOI
    long_range = [left, right] # Define boundaries of longitude in AOI
    mPerPixel = 2*alt*math.tan(math.radians(65)/2)/1920
    degree_per_meter = 1/111139
    pixel_width_map = (long_range[1] - long_range[0]) / (mPerPixel * degree_per_meter)
    pixel_height_map = (lat_range[1] - lat_range[0]) / (mPerPixel * degree_per_meter)
    
    # Read in image of map from "stitching.py" into image object
    
    stitched_map = Image.open(stitched_map)
    
    # Loop through each GPS coordinate, for each identification, evaluate 
    # pixel location of centroid of target/tarp and overlay text box with 
    # strings containing GPS coordinate position and identification
    
    for i in range(0, len(identifications)):
        
        GPS_coords.append(GPS_coords_lat[i] + GPS_coords_long[i])
        
        if (identifications[i] != "none"):
            
            # Calculate pixel location of centroid of target/tarp and overlay text
            # box with strings containing GPS coordinate position and identification
            
            ''' First check the lat and long range of mapped region, and then determine
            corresponding pixel width and height of fully mapped image. then, 
            using this range, determine corresponding pixel location of identifications
            given ratio of GPS coordinate of identification to GPS coordinate range
            of width/height '''
            
            
            
            GPS_long_factor = GPS_coords_long[i] / long_range
            GPS_lat_factor = GPS_coords_lat[i] / lat_range
            
            pixel_x = GPS_long_factor * pixel_width_map
            pixel_y = GPS_lat_factor * pixel_height_map
            
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