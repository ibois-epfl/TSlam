import cv2
import numpy as np

cap = cv2.VideoCapture(0)

#Check whether user selected camera is opened successfully.

if not (cap.isOpened()):
    print("Could not open video device")


while(True):

    # Capture frame-by-frame
    ret, frame = cap.read()

    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_HSV_threshold = cv2.inRange(frame_HSV, (0, 0, 50), (75, 100, 255))

    mask = frame_HSV_threshold.copy()

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(frame_gray, 155, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
	
    f_copy = frame.copy()
    
    cv2.drawContours(image=mask, contours=contours, contourIdx=-1, color=(255, 255, 255), thickness=cv2.FILLED)
    cv2.drawContours(image=f_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

    # Creating kernel
    kernel = np.ones((5, 5), np.uint8)
    
    # Using cv2.erode() method 
    mask = cv2.erode(mask, kernel) 
    mask = cv2.dilate(mask, kernel, iterations=3)
    f_copy[mask == 0] = 0

    # Display the resulting frame
    cv2.imshow("mask",mask)
    cv2.imshow("preview",f_copy)

    #Waits for a user input to quit the application
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

# vector::_M_range_check: __n (which is 1675162384) >= this->size() (which is 0)