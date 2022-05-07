import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
# for pixhawk
import dronekit
from dronekit import connect
from dronekit import VehicleMode
import time



def captureLiveVideoTest(vehicle):
    cap = cv2.VideoCapture(0)
    frame_width = 1920
    frame_height = 1080
    cap.set(3,frame_width)
    cap.set(4,frame_height)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    
    count = 10000
    f = open('Roll_Values.txt'+str(time.time_ns()), 'a')
    while cap.isOpened():
        # pull Roll
        roll = vehicle.attitude.roll
    
    while(count <= 10300):
        ret, frame = cap.read()
    
        if ret == True: 
    
            # Write the frame into the
            cv2.write('/Desktop/outputImagesRoll/frame'+str(count)+'.png')
            f.write('Roll value: '+str(math.degrees(roll)) + 'Frame Number: '+str(count)+ '\n')
            time.sleep(1)
    
        # Break the loop
        else:
            break

        count += 1
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
        
    f.close()


def main():
    #connection to pixhawk
    vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=921600)
    captureLiveVideoTest(vehicle)

if __name__ == "__main__":
    main()