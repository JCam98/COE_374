import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
# for pixhawk
import dronekit
from dronekit import connect
from dronekit import VehicleMode
import time
from PIL import Image, ImageFont, ImageDraw


def captureLiveVideoTest(vehicle, maxRoll, timeToRun):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    count = 10000
    startTime = time.time()
    gps_coord_filename = 'GPS_coords.txt'+str(startTime)
    f = open(gps_coord_filename, 'a')

    # run ATR for timeToRun amount of time
    while (time.time() - startTime) < timeToRun:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Undisort image frame 
        
        frame = undistort_frames(frame)
        
        # pull GPS
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        alt = vehicle.location.global_relative_frame.alt
        roll = vehicle.attitude.roll

        # save image if plane's roll value is less than our set maxRoll
        if roll <= maxRoll:
            cv2.imwrite("mapStichImg/frame" + str(count) + ".png",frame)

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        target_type, target_lat, target_lon  = featRecog(frame, count, lat, lon, alt)
        f.write(target_type+'  lat='+str(target_lat)+' , lon='+str(target_lon)+'\n')
        count += 1


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
        
    f.close()

def undistort_frames(frame):
    
    # Define Camera Calibration Parameters from "camera_calibration_standard.py"
        
    dist = np.array([[-0.50674759,  0.23775016, -0.01112213, -0.00096096,  0.17855823]])
    mtx = np.array([[1882.35, 0, 959.767], [0, 1875.87, 575.389]])
    
    h,  w = frame.shape[:2] # Return frame size

    # Obtain new camera matrix for removing distortion from input image

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # # Undistortion Method: Using remapping

    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    frame = cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)
  
    #Crop the image
    x, y, w, h = roi
    frame = frame[y:y+h, x:x+w]

    return frame

def featRecog(frame, count, lat, lon, alt):
    ## Read

    img = frame
    ## convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, (100, 25, 25), (125,255,255))

    ## slice the green
    imask = mask>0
    green = np.zeros_like(img, np.uint8)
    green[imask] = img[imask]


    ret,thresh1 = cv2.threshold(green,200,255,cv2.THRESH_BINARY)
    gray = cv2.cvtColor(thresh1, cv2.COLOR_BGR2GRAY)

    thresh = 20
    im_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
    #cv2.imwrite('testImages/bw_image.png', im_bw)
    # find and draw contours

    contours = cv2.findContours(im_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Find the contour with the largest area

    if contours:
        conts = max(contours, key=cv2.contourArea)

        # Find the bounding rectangle around the max contour (x,y) is top left coord, (w,h) is width and height

        x,y,w,h = cv2.boundingRect(conts)

        print(str(h) + " "+ str(count))
        # checking to make sure the bounding box that is cropped is in range of what a face would be
        if h > 30 and h < 150:
        
            #cv2.imwrite("outputImages/outlined_image.png",img)

            
            # Crop the image to the bounding rectangle only

            crop_img = img[y:y+h, x:x+w]
            crop_bw = im_bw[y:y+h, x:x+w]

            cv2.imwrite("outputImages/zoomed_in.png",crop_img)
            cv2.imwrite("outputImages/zoomed_inbw.png", crop_bw)

            im = cv2.imread('outputImages/zoomed_inbw.png')
            invert = cv2.bitwise_not(im)
            imgray = cv2.cvtColor(invert, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(imgray, 127, 255, 0)
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            c = max(contours, key=cv2.contourArea)

            (x,y),radius = cv2.minEnclosingCircle(c)
            center = (int(x),int(y))
            radius = int(radius)

            x,y,w,h = cv2.boundingRect(c)
            try:
                if w > 30:
                    face = crop_img[y:y+h, x:x+w]
                    cv2.imwrite("outputImages/face.png",face)

                    # SIFT Feature Matching

                    img1 = cv2.imread('inputImages/querySmile.png',cv2.IMREAD_GRAYSCALE) # queryImage
                    img2 = cv2.imread('outputImages/face.png',cv2.IMREAD_GRAYSCALE) # trainImage
                    img3 = cv2.imread('inputImages/queryFrowny.png',cv2.IMREAD_GRAYSCALE) # queryImage


                    # Initiate SIFT detector
                    sift = cv2.SIFT_create()

                    # find the keypoints and descriptors with SIFT
                    kp1, des1 = sift.detectAndCompute(img1,None)
                    kp2, des2 = sift.detectAndCompute(img2,None)
                    kp3, des3 = sift.detectAndCompute(img3,None)

                    # FLANN parameters
                    FLANN_INDEX_KDTREE = 1
                    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
                    search_params = dict(checks=100)   # or pass empty dictionary
                    flann = cv2.FlannBasedMatcher(index_params,search_params)
                    
                    matchesSmiley = flann.knnMatch(des1,des2,k=2)
                    matchesFrowny= flann.knnMatch(des3,des2,k=2)

                    # Need to draw only good matches, so create a mask
                    matchesSmileyMask = [[0,0] for i in range(len(matchesSmiley))]
                    matchesFrownyMask = [[0,0] for i in range(len(matchesFrowny))]



                    # ratio test as per Lowe's paper Smiley
                    countGoodMatchesSmiley = 0
                    for i,(m,n) in enumerate(matchesSmiley):
                        if m.distance < 0.8*n.distance:
                            matchesSmileyMask[i]=[1,0]
                            countGoodMatchesSmiley += 1
                        draw_params_smiley = dict(matchColor = (0,255,0),
                                    singlePointColor = (255,0,0),
                                    matchesMask = matchesSmileyMask,
                                    flags = cv2.DrawMatchesFlags_DEFAULT)
                    img4 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matchesSmiley,None,**draw_params_smiley)

                    # ratio test as per Lowe's paper Frowny
                    countGoodMatchesFrowny = 0
                    for i,(m,n) in enumerate(matchesFrowny):
                        if m.distance < 0.8*n.distance:
                            matchesFrownyMask[i]=[1,0]
                            countGoodMatchesFrowny += 1
                        draw_params_frowny = dict(matchColor = (0,255,0),
                                    singlePointColor = (255,0,0),
                        matchesMask = matchesFrownyMask,
                                    flags = cv2.DrawMatchesFlags_DEFAULT)
                    img5 = cv2.drawMatchesKnn(img3,kp3,img2,kp2,matchesFrowny,None,**draw_params_frowny)

                    if countGoodMatchesSmiley >= 4 or countGoodMatchesFrowny >= 4:
                        if countGoodMatchesSmiley > countGoodMatchesFrowny:
                            print('Smiley Face!!')
                            M = cv2.moments(conts)
                            if M['m00'] != 0:
                                cx = int(M['m10']/M['m00'])
                                cy = int(M['m01']/M['m00'])
                                print('cx = '+str(cx) +' , cy = '+str(cy))
                                target_lat, target_lon = calcGPS(lat, lon, cx, cy, alt)
                                cv2.imwrite('/home/pi/Desktop/outputImagesTest/Smiley'+str(count)+'.png', frame)
                                return 'Smiley_Face'+str(count), target_lat, target_lon

                        else:
                            print('Frowny Face!!')
                            M = cv2.moments(conts)
                            if M['m00'] != 0:
                                cx = int(M['m10']/M['m00'])
                                cy = int(M['m01']/M['m00'])
                                print('cx = '+str(cx) +' , cy = '+str(cy))
                                target_lat, target_lon = calcGPS(lat, lon, cx, cy, alt)
                                cv2.imwrite('/home/pi/Desktop/outputImagesTest/Frowny'+str(count)+'.png', frame)
                                return 'Frowny Face'+str(count), target_lat, target_lon

                    else:
                        print('Tarp')
                        M = cv2.moments(conts)
                        if M['m00'] != 0:
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                            print('cx = '+str(cx) +' , cy = '+str(cy))
                            target_lat, target_lon = calcGPS(lat, lon, cx, cy, alt)
                            print('lat= ' +str(target_lat)+ ' lon='+str(target_lon))
                            return 'tarp'+str(count),target_lat, target_lon



                    cv2.imwrite("outputImages/SIFT_test" + str(count) + ".png",img4)
                    cv2.imwrite("outputImages/SIFT_test" + str(count) + ".png",img5)
            except:
                pass

    return 'none', 0, 0 



def calcGPS(gpsLat, gpsLon, xpoint, ypoint, alt):
    mPerPixel = 2*alt*math.tan(math.radians(65)/2)/640
    lat_center = 240
    lon_center = 320

    convertDeg = 1/111139

    xChange = (xpoint - lon_center)*mPerPixel*convertDeg
    yChange = (ypoint - lat_center)*mPerPixel*convertDeg

    newGpsLon = gpsLon + xChange
    newGpsLat = gpsLat + yChange

    return newGpsLat, newGpsLon

''' Define function for generating static digital map with GPS coordinate and 
target/tarp identifications '''

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
            
            # pixel_x = (xChange/(mPerPixel*convertDeg)) + lon_center
            # pixel_y = (yChange/(mPerPixel*convertDeg)) + lat_center
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