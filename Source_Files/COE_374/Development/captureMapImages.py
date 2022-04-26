import undistort_frames as uf
import time
import resize
import cv2


def mapImages(vehicle):
    cap = cv2.VideoCapture(0)
    lapCount = 0
    wayList = [1]
    while lapCount < 3:
        newWay = vehicle.commands.next
        if newWay == 7 and wayList[len(wayList)-1] != newWay
            lapCount = lapCount + 1
        if nextWay >= 8 or nextWay <= 2:
            if not cap.isOpened():
                print("Cannot open camera")
                exit()
            count = 10001

            # Capture frame-by-frame
            ret, frame = cap.read()
            
            # Undisort image frame 
            frame = uf.undistort_frames(frame)

            # Undistorted frame is resized to 75% of its original size
            frame = resize.resize(frame,count)

            # Save to directory
            cv2.imwrite('images/image_dir'+ str(lapCount) +'/frame' + str(count) +'.jpg', frame)

            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            
            count += 1

        wayList.append(newWay)
    cap.release()
    cv2.destroyAllWindows()