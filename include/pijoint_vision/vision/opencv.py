from math import atan2, cos, pi, sin, sqrt
import math
import cv2
import numpy as np

DEFAULT_SMIN = 170
SMIN_VALUE = {
  3: 70
}



def getOrientation(pts, img):
    ## [pca]
    # Construct a buffer used by the pca analysis
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i, 0] = pts[i, 0, 0]
        data_pts[i, 1] = pts[i, 0, 1]

    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)

    # Store the center of the object
    cntr = (int(mean[0, 0]), int(mean[0, 1]))
    ## [pca]

    ## [visualization]
    # Draw the principal components
    cv2.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (
    cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0], cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])
    p2 = (
    cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0], cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])
   
    angle = atan2(eigenvectors[0, 1], eigenvectors[0, 0])  # orientation in radians

    return cntr, angle



def mask(img, cl):
    original = img.copy()   
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([0, SMIN_VALUE.get(cl, 170), 0])
    upper = np.array([179, 255, 255])
    return cv2.inRange(hsv, lower, upper)



def angle(img, cl):
    cnts,_ = cv2.findContours(mask(img, cl), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)

    cv2.drawContours(img,cnts, 0, (255,0,0), 1)

    center, angle = getOrientation(cnts[0], img)
    angle = -int(np.rad2deg(angle))

    if angle > 180:
        angle -=180

    if angle < -180:
        angle +=180
    
    if angle < 0:
        angle = 180 + angle
    # TODO pythonic refactor

    if angle > 90:
        angle -=90
    
    else:
        angle +=90

    return center, np.deg2rad(angle)

