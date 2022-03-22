import cv2 as cv
import numpy as np
import os

# given a list of [x, y, w, h] rectangles and a canvas image to draw on, return an image with
# all of those rectangles drawn
def draw_rectangles(haystack_img, rectangles):
    # these colors are actually BGR
    line_color = (0, 255, 0)
    line_type = cv.LINE_4

    for (x, y, w, h) in rectangles:
        # determine the box positions
        top_left = (x, y)
        bottom_right = (x + w, y + h)
        # draw the box
        cv.rectangle(haystack_img, top_left, bottom_right, line_color, lineType=line_type)

    return haystack_img

def main():
    # load the trained model
    cascade_limestone = cv.CascadeClassifier('cascade\cascade.xml')


    # do object detection
    im = cv.imread("pos_smiley/image18.png")
    rectangles = cascade_limestone.detectMultiScale(im)
    

    # draw the detection results onto the original image
    detection_image = draw_rectangles(im, rectangles)

    

    # display the images
    cv.imshow('Matches', detection_image)


    print('Done.')

if __name__ == "__main__":
    main()
