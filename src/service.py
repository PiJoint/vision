
from functools import partial
from pijoint_vision.srv import ObjectDetection,ObjectDetectionResponse
from pijoint_vision.msg import RGBD, Object,Pose,Rotation
from geometry_msgs.msg import Point
import rospy
import cv2
import numpy as np
from math import sqrt

from cv_bridge import CvBridge, CvBridgeError

from pijoint_vision.ai import Model
from pijoint_vision.vision import pixel2cloud, angle
from pijoint_vision.vision.utils import trw



bridge = CvBridge()

classifier = Model('src/last.pt', 0.70)
up_classifier = Model('src/up_and_down.pt', 0.50)

def pose(img0, ob_c, depth, box):

    depth, point_cloud = depth
    x,y,x1,y1  = box


    cropped = img0[int(y):int(y1), int(x):int(x1)]
    ob = up_classifier.detect_object(cropped)
    
    pitch = 0
    roll = 0
    for o in ob:
        c, _ = o
        # TODO 
        # - get pitch
        # - get roll
        
    (cx, cy), yaw = angle(cropped, ob_c)
    px,py,pz = pixel2cloud(point_cloud, cx+x,cy+y)

    rospy.loginfo("\tCenter/Angle %f %f %f %f" % (px,py,pz,yaw))
        
    return Pose(
        Point(*trw(px,py,pz)),
        Rotation(yaw, pitch, roll)
    ), cropped


def object_detection(req):
    rospy.loginfo("Starting detection...")
    rgbd = rospy.wait_for_message('/pjoint/_internal/camera', RGBD)
    
    try:
        # Convert your ROS Image message to OpenCV2
        left = bridge.imgmsg_to_cv2(rgbd.image, 'bgr8')
        depth = bridge.imgmsg_to_cv2(rgbd.depth)
        point_cloud = rgbd.point_cloud

        cv2.resize(depth,(1920,1080))
        cv2.resize(left,(1920,1080))

    except CvBridgeError as e:
        return ObjectDetectionResponse(False,0, [])
    else:
        # DETECT AND PUBLISH
        objects = []
        ob = classifier.detect_object(left)
        
        for o in ob:
            
            c, (x,y,x1,y1) = o
            rospy.loginfo("Found %d" % c)

            rotation, cropped = pose(left, c, (depth, point_cloud), (x,y,x1,y1))
            objects.append(
                Object(c, rotation, rgbd.image, bridge.cv2_to_imgmsg(cropped))
            )
    return ObjectDetectionResponse(True,len(objects), objects)
            

def init():
    rospy.loginfo("Loading models...")

  
    rospy.loginfo("Fused...")

    rospy.loginfo("Loading services...")
    rospy.Service('object_detection', ObjectDetection,object_detection)



