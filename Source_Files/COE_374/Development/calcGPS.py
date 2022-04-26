''' Description: The function, "calcGPS()" is used to evaluate the GPS coordinates
the centroid of targets/tarps and the centroid of area of all images frames 
stitched together in "stitching.py" for the purpose of generating the map for 
the entire AOI. This function will be invoked in "main.py" '''

import math

def calcGPS(gpsLat, gpsLon, xpoint, ypoint, alt):
    mPerPixel = 2*alt*math.tan(math.radians(65)/2)/1376
    lat_center = 349.5
    lon_center = 688

    convertDeg = 1/111139

    xChange = (xpoint - lon_center)*mPerPixel*convertDeg
    yChange = (ypoint - lat_center)*mPerPixel*convertDeg

    newGpsLon = gpsLon + xChange
    newGpsLat = gpsLat + yChange

    return newGpsLat, newGpsLon
