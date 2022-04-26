import undistort_frames as uf
import time
import resize
import cv2


def mapImages(directorynum):
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    count = 10001

    while cap.isOpened():
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Undisort image frame 
        frame = uf.undistort_frames(frame)

        # Undistorted frame is resized to 75% of its original size
        frame = resize.resize(frame,count)

        # Save to directory
        cv2.imwrite('images/image_dir' + str(directorynum) + '/frame' + str(count) +'.jpg', frame)

        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        
        count += 1

    cap.release()
    cv2.destroyAllWindows()