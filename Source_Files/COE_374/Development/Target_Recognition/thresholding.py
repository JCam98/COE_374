import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
# for pixhawk
#import dronekit
#from dronekit import connect
#from dronekit import VehicleMode
import time



def captureLiveVideoTest():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    count = 0
    f = open('GPS_coords.txt'+str(time.time_ns()), 'a')
    while cap.isOpened():
        # Capture frame-by-frame
        ret, frame = cap.read()
        #cv2.imwrite('frame' +str(count)+'.png', frame)
        # pull GPS
#        lat = vehicle.location.global_relative_frame.lat
#        lon = vehicle.location.global_relative_frame.lon
#        alt = vehicle.location.global_relative_frame.alt

        lat = 30.32420
        lon = -97.60313
        alt = 76.2
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        target_type, target_lat, target_lon  = featRecog(frame, count, lat, lon, alt)
        f.write(target_type+'  lat='+str(target_lat)+' , lon='+str(target_lon)+'\n')
        # Display the resulting frame
        #cv2.imwrite('frame' +str(count)+'.png', gray)
        count += 1


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
        
    f.close()





def featRecog(frame, count, lat, lon, alt):
    ## Read
    #img = cv2.imread(frameName)
    img = frame
    ## convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    ## mask of green (36,25,25) ~ (86, 255,255)
    # mask = cv2.inRange(hsv, (36, 25, 25), (86, 255,255))
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
            if w > 5:
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

        # ratio test as per Lowe's paper Smiley
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
                    #target_lat , target_lon = xy2LatLon(lat, lon, 1, 640, 480, cx, cy)
                    #target_dict = {'smiley_face'+str(count):[target_lat, target_lon]}
                    target_lat, target_lon = calcGPS(lat, lon, cx, cy, alt)
                    cv2.imwrite('/home/pi/Desktop/outputImagesTest/Smiley'+str(count)+'.png', frame)
                    return 'Smiley_Face'+str(count), target_lat, target_lon
                # plt.imshow(img4,),plt.show()
            else:
                print('Frowny Face!!')
                M = cv2.moments(conts)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    print('cx = '+str(cx) +' , cy = '+str(cy))
                    #target_lat , target_lon = xy2LatLon(lat, lon, 1, 640, 480, cx, cy)
                    #target_dict = {'frowny_face'+str(count):[target_lat, target_lon]}
                    target_lat, target_lon = calcGPS(lat, lon, cx, cy, alt)
                    cv2.imwrite('/home/pi/Desktop/outputImagesTest/Frowny'+str(count)+'.png', frame)
                    return 'Frowny Face'+str(count), target_lat, target_lon
                # plt.imshow(img5,),plt.show()
        else:
            print('Tarp')
            M = cv2.moments(conts)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print('cx = '+str(cx) +' , cy = '+str(cy))
                #target_lat , target_lon = xy2LatLon(lat, lon, 1, 640, 480, cx, cy)
                #target_dict = {'tarp'+str(count):[target_lat, target_lon]}
                target_lat, target_lon = calcGPS(lat, lon, cx, cy, alt)
                print('lat= ' +str(target_lat)+ ' lon='+str(target_lon))
                return 'tarp'+str(count),target_lat, target_lon
            # plt.imshow(img4,),plt.show()
            # plt.imshow(img5,),plt.show()



        cv2.imwrite("outputImages/SIFT_test" + str(count) + ".png",img4)
        cv2.imwrite("outputImages/SIFT_test" + str(count) + ".png",img5)

    return 'none', 0, 0   


def calcGPS(gpsLat, gpsLon, xpoint, ypoint, alt):
    mPerPixel = 2*alt*math.tan(math.rad(65)/2)/640
    lat_center = 240
    lon_center = 320

    convertDeg = 1/111139

    xChange = (xpoint - lon_center)*mPerPixel*convertDeg
    yChange = (ypoint - lat_center)*mPerPixel*convertDeg

    newGpsLon = gpsLon + xChange
    newGpsLat = gpsLat + yChange

    return newGpsLat, newGpsLon

def main():
    #connection to pixhawk
    #vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=921600)

    # sec = 0
    # frameRate = 3 #//it will capture image in each 1 second
    # count=0
    # success = captureFrames("../Input_Data/Training_videos/flight_test_Trim.mp4", sec, count)
    # while success:
    #     sec = sec + frameRate
    #     sec = round(sec, 2)
    #     success = captureFrames("../Input_Data/Training_videos/flight_test_Trim.mp4", sec, count)
    #     count = count + 1
    captureLiveVideoTest()

if __name__ == "__main__":
    main()
