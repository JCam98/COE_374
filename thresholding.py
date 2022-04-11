import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import os


def captureFrames(pathToVid, sec, count):
    # Read the video from specified path
    cam = cv2.VideoCapture(pathToVid)

    cam.set(cv2.CAP_PROP_POS_MSEC,sec*1000)
    hasFrames,image = cam.read()
    if hasFrames:
        frameName = "./data/image"+str(count)+".jpg" 
        cv2.imwrite(frameName, image)     # save frame as JPG file
        featRecog(frameName, count)
    return hasFrames




def featRecog(frameName, count):
    ## Read
    # img = cv2.imread('../Input_Data/Training_Images/testField.png')
    img = cv2.imread(frameName)
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
    cv2.imwrite('testImages/bw_image.png', im_bw)
    # find and draw contours

    contours = cv2.findContours(im_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Find the contour with the largest area

    if contours:
        c = max(contours, key=cv2.contourArea)

        # Find the bounding rectangle around the max contour (x,y) is top left coord, (w,h) is width and height

        x,y,w,h = cv2.boundingRect(c)

        print(str(h) + " "+ str(count))
        # checking to make sure the bounding box that is cropped is in range of what a face would be
        if h > 30 and h < 150:
        
            cv2.imwrite("outputImages/outlined_image.png",img)

            
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
                try: 
                    face = crop_img[y:y+h, x:x+w]

                    cv2.imwrite("outputImages/face.png",face)

                    # SIFT Feature Matching

                    img1 = cv2.imread('inputImages/querySmile.png',cv2.IMREAD_GRAYSCALE) # queryImage
                    img2 = cv2.imread('outputImages/face.png',cv2.IMREAD_GRAYSCALE) # trainImage


                    # Initiate SIFT detector
                    sift = cv2.SIFT_create()

                    # find the keypoints and descriptors with SIFT
                    kp1, des1 = sift.detectAndCompute(img1,None)
                    kp2, des2 = sift.detectAndCompute(img2,None)

                    # FLANN parameters
                    FLANN_INDEX_KDTREE = 1
                    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
                    search_params = dict(checks=100)   # or pass empty dictionary
                    flann = cv2.FlannBasedMatcher(index_params,search_params)
                    matches = flann.knnMatch(des1,des2,k=2)

                    # Need to draw only good matches, so create a mask
                    matchesMask = [[0,0] for i in range(len(matches))]
                    
                    

                    # ratio test as per Lowe's paper
                    countGoodMatches = 0
                    for i,(m,n) in enumerate(matches):
                        if m.distance < 0.8*n.distance:
                            matchesMask[i]=[1,0]
                            countGoodMatches += 1
                    draw_params = dict(matchColor = (0,255,0),
                                    singlePointColor = (255,0,0),
                                    matchesMask = matchesMask,
                                    flags = cv2.DrawMatchesFlags_DEFAULT)
                    img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)

                    if countGoodMatches >= 4:
                        print('Smiley Face!!')
                    else:
                        print('Tarp')

                    plt.imshow(img3,),plt.show()

                    cv2.imwrite("outputImages/SIFT_test" + str(count) + ".png",img3)
                except:
                    pass


def main():
    sec = 0
    frameRate = 7 #//it will capture image in each 1 second
    count=0
    success = captureFrames("../Input_Data/Training_videos/flight_test_Trim.mp4", sec, count)
    while success:
        sec = sec + frameRate
        sec = round(sec, 2)
        success = captureFrames("../Input_Data/Training_videos/flight_test_Trim.mp4", sec, count)
        count = count + 1


if __name__ == "__main__":
    main()