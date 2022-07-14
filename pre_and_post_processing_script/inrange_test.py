import cv2 as cv2
import argparse
import numpy as np
max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)
parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()
cap = cv2.VideoCapture(args.camera)
cv2.namedWindow(window_capture_name)
cv2.namedWindow(window_detection_name)
cv2.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)

out_ori = cv2.VideoWriter('output_original.mp4', 0x7634706d , 20.0, (640,480))
out_mask = cv2.VideoWriter('output_mask.mp4', 0x7634706d , 20.0, (640,480))

flag = False
while True:
    
    ret, frame = cap.read()
    if frame is None:
        break
    

    # contour mask
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(frame_gray, 155, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
    contours_mask = np.zeros_like(frame_gray)
    
    cv2.drawContours(image=contours_mask, contours=contours, contourIdx=-1, color=(255, 255, 255), thickness=cv2.FILLED)

    # HSV mask
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    HSV_mask = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    # Combine
    mask = np.logical_or(contours_mask, HSV_mask)

    # Creating kernel
    kernel = np.ones((5, 5), np.uint8)
    
    # Using cv2.erode() method 
    contours_mask = cv2.erode(contours_mask, kernel, iterations=3)
    contours_mask = cv2.dilate(contours_mask, kernel, iterations=3)

    cropped = frame.copy()
    cropped[:, :, 0][mask[:, :] == 0] = 0
    cropped[:, :, 1][mask[:, :] == 0] = 0
    cropped[:, :, 2][mask[:, :] == 0] = 0

    #cv2.drawContours(image=cropped, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    cv2.imshow(window_capture_name, frame)
    cv2.imshow("window_HSV_mask", HSV_mask)
    cv2.imshow("window_contour_mask", contours_mask)

    cv2.imshow(window_detection_name, cropped)
    
    
    if flag:
        out_ori.write(frame)
        out_mask.write(cropped)

    key = cv2.waitKey(30)
    if key == ord('q') or key == 27:
        break
    if key == ord('r'):
        flag = not flag
        print("Recording: ", flag)

cap.release()
out_mask.release()
out_ori.release()