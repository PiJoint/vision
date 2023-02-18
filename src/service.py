import sys
from functools import partial
import rospy
import cv2
import numpy as np
from math import sqrt
import sys
from yolov7 import models

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Point

from pijoint_vision.srv import ObjectDetection,ObjectDetectionResponse
from pijoint_vision.msg import RGBD, Object,Pose,Rotation
from pijoint_vision.ai import Model
from pijoint_vision.vision import pixel2cloud, MegaBloks
from pijoint_vision.vision.utils import trw

sys.modules['models'] = models

bridge = CvBridge()

classifier = Model('src/models/last.pt', 0.70)
up_classifier = Model('src/models/up_and_down.pt', 0.30)


def group_position(img):
    def area(x,y):
        return float(0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1))))
    
    ob = up_classifier.detect_object(img)
    matches = sorted([
        (c, area(
            (box[0], box[2]), (box[1], box[3])
        ), conf) for c,box, conf in ob
    ], reverse=True, key=lambda x: (x[1], x[2]) )
    if len(matches) > 0:
        return matches[0][0]
    else: 
        None


def pose(img, ob_c, point_cloud, box):
    x,y,x1,y1  = box

    cropped = img[int(y):int(y1), int(x):int(x1)]
    blok = MegaBloks(cropped, (ob_c, group_position(cropped)))

    (cx, cy) = blok.center
    (tx, ty) = blok.piolini
    px,py,pz = pixel2cloud(point_cloud, cx+x,cy+y)
    tx,ty,tz = pixel2cloud(point_cloud, tx+x,ty+y)

    return Pose(
        Point(*trw(px,py,pz)),Point(*trw(tx,ty,tz)), blok.yaw, blok.ground
    ), blok.debug_image


def object_detection(req):
    rospy.loginfo("Starting detection...")
    rgbd = rospy.wait_for_message('/pjoint/_internal/camera', RGBD)
    
    try:
        # Convert your ROS Image message to OpenCV2
        left = bridge.imgmsg_to_cv2(rgbd.image, 'bgr8')
        point_cloud = rgbd.point_cloud
        cv2.resize(left,(1920,1080))

    except CvBridgeError as e:
        return ObjectDetectionResponse(False,0, [])
    else:
        # DETECT AND PUBLISH
        objects = []
        ob = classifier.detect_object(left)
        
        for o in ob:
            
            c, (x,y,x1,y1),_ = o
            rospy.loginfo("Found %d" % c)

            rotation, cropped = pose(left, c, point_cloud, (x,y,x1,y1))
            objects.append(
                Object(c, rotation, rgbd.image, bridge.cv2_to_imgmsg(cropped))
            )
    return ObjectDetectionResponse(True,len(objects), objects)
            

def init():
    rospy.loginfo("Loading models...")

  
    rospy.loginfo("Fused...")

    rospy.loginfo("Loading services...")
    rospy.Service('object_detection', ObjectDetection,object_detection)



