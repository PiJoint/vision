#!/usr/bin/env python
"""!

    This file is part of PiJoint.

    @package pijoint_vision vision
    @author: Alessandro Mizzaro
    @version: 1.0.0

"""
from math import atan2, cos, pi, sin, sqrt
import math
import cv2
import os
import numpy as np

from enum import IntEnum

## Default mask value
DEFULT_HSV= (0,170,0)  
## violet mask value
HSV_VALUE = { 
  3: (0,70,0)
}

class Ground(IntEnum):
    """!
    Ground enum
    """
    ## DOWN is class 0
    DOWN = 0
    ## UP is class 1
    UP = 1



def mask(img, hsvs): 
    """!
    Return the mask of the image. The mask is the image with only the color of the object
    @param img: image
    @param hsvs: hsv values
    @return: mask

    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([hsvs[0], hsvs[1], hsvs[2]])
    upper = np.array([179, 255, 255])
    return cv2.inRange(hsv, lower, upper)


def drawAxis(img, p_, q_, colour, scale):
    """!
    Utils function to draw an arrow with eigenvalues and eigenvectors. In place edit
    @param img: image
    @param p_: point
    @param q_: point
    @param colour: color
    @param scale: scale
    """
    p = list(p_)
    q = list(q_)
    
    angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)



class MegaBloks:
    """!
    Class to manage the MegaBloks

    """

    _TEMPLATE = os.path.dirname(__file__) + '/templates/piolino.jpg'

    def __init__(self, img, c) -> None:
        """!
        Constructor
        @param img: image
        @param c: class

        """
        ## Object class
        self.object, self.ground_position = c
        ## Image
        self.img = img
        ## Ground position
        self.ground = self.ground_position == Ground.DOWN
        hsvs = HSV_VALUE.get(self.object, DEFULT_HSV)

        
        cnts,_ = cv2.findContours(mask(img, hsvs), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)

        ## Contour
        self.cnt = cnts[0]

        self._calc_orientation()
        self.sift()

    
    def _calc_orientation(self):
        """!
        Calculate the yaw angle of the object with mask and PCA method

        """
        pts = self.cnt
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i, 0] = pts[i, 0, 0]
            data_pts[i, 1] = pts[i, 0, 1]

        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
        
        index  = np.argmax(eigenvalues)
        angle = math.atan2(-eigenvectors[index, 1], eigenvectors[index, 0])  # orientation in radiants
        if angle < 0:
            angle = -(math.pi + angle)

        ## Yaw angle
        self.yaw = angle
        ## Eigenvectors
        self.eigenvectors = (eigenvectors[index, 0], eigenvectors[index, 1])
        ## Eigenvalues
        self.eigenvalue = eigenvalues[index] 
        ## Center
        self.center = (int(mean[0, 0]), int(mean[0, 1]))

    def sift(self):
        """!
        Calculate the skeleton of the object with SIFT method

        """
        template = cv2.imread(self._TEMPLATE)
        sift = cv2.xfeatures2d.SIFT_create()

        block = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

        t_keypoint, t_descriptors = sift.detectAndCompute(template, None)
        o_keypoint, o_descriptors = sift.detectAndCompute(block, None)

        bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
        matches = bf.match(t_descriptors,o_descriptors)
        matches = sorted(matches, key = lambda x:x.distance)

        ## Skeleton
        self.skeleton = []

        for match in matches:
            x, y = o_keypoint[match.trainIdx].pt
            self.skeleton.append((x,y))
        
        ## Piolino
        self.piolini = list(map(int, np.mean(self.skeleton, axis=0)))
        
    
    @property
    def debug_image(self):
        """!
        Return the debug image with the skeleton and the orientation
        """
        img = self.img.copy()
        cntr = self.center

        cv2.drawContours(img,[self.cnt], 0, (255,0,0), 1)
        drawAxis(
            img, cntr, 
            (cntr[0] + 0.02 * self.eigenvectors[0] * self.eigenvalue, cntr[1] + 0.02 * self.eigenvectors[1] * self.eigenvalue), (255, 255, 0), 1
        )

        for p in self.skeleton:
            x,y = p
            img = cv2.line(img, (int(x),int(y)), cntr, (0,255,0), 1)
            img = cv2.circle(img, (int(x),int(y)), 5, (0, 255, 0), 1)

        img = cv2.circle(img, cntr, 5, (0,0,255), 1)
        img = cv2.circle(img, self.piolini, 5, (0,0,255), 1)
        return img
        
