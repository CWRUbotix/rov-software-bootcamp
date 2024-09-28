import cv2
import numpy as np

if __name__ == '__main__':
    frame = cv2.imread('image_docking_2.png')
    # cv2.imshow('This is a flower', frame)
    # cv2.waitKey(0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    bottom_red_lower = np.array([0, 50, 50])
    bottom_red_upper = np.array([10, 255, 255])
    top_red_lower = np.array([170, 50, 50])
    top_red_upper = np.array([180, 255, 255])
    
    bottom_mask = cv2.inRange( hsv_frame, bottom_red_lower, bottom_red_upper )
    top_mask = cv2.inRange(hsv_frame, top_red_lower, top_red_upper)
    
    mask = bottom_mask + top_mask

    contours, _ = cv2.findContours(
        mask.copy(),
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE
    )

    red_contour = contours[0]
    if len(contours) > 0:
        biggest_contour = max(contours, key=cv2.contourArea)
        red_contour = biggest_contour

    x, y, w, h = cv2.boundingRect(red_contour)

    cv2.rectangle(mask, (x, y), (x + w, y + h), (100, 255, 255), 2)

    cv2.imshow('This is the mask', mask)
    cv2.waitKey(0)