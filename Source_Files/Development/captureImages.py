
import undistort_frames as uf
import time
#import resize
import cv2


def mapImages(vehicle, output):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    count = 10001

    lapCount = 0
    inc = 0
    oldWay = 1
    print('before loop', file=output)
    while lapCount < 3:
        if inc >= 12:
            inc = 0
        # CHANGE THIS
        #newWay = vehicle.commands.next
        print('lapCount' + str(lapCount), file=output)
        print('inc: '+str(inc), file=output)
        inc = inc + 1
        vehicle.commands.next = inc
        newWay = vehicle.commands.next
        print('new waypoint:', file=output)
        print('old waypoint:', file=output)
        if newWay == 7 and oldWay != newWay:
            print('increment lap', file=output)
            lapCount = lapCount + 1
        if newWay >= 8 or newWay <= 2:
            print('start taking map images', file=output)
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            # Undisort image frame 
            #frame = uf.undistort_frames(frame)

            # Undistorted frame is resized to 75% of its original size
            #frame = resize.resize(frame,count)

            # Save to directory
            cv2.imwrite('~/Desktop/images/image_dir'+ 'frame' + str(count) + '.jpg', frame)

            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            
            count += 1

        oldWay = newWay
    cap.release()
    cv2.destroyAllWindows()
