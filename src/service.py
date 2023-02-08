
from functools import partial
from pijoint_vision.srv import ObjectDetection,ObjectDetectionResponse
from pijoint_vision.msg import RGBD, Object,Pose,Rotation
from geometry_msgs.msg import Point
import rospy
import cv2
from math import sqrt

from cv_bridge import CvBridge, CvBridgeError
from pijoint_vision.ai import Model
from pijoint_vision.vision import pixel2cloud
from pijoint_vision.vision.utils import trw


bridge = CvBridge()

classifier = Model('src/best.pt', 0.30)
up_classifier = Model('src/up_and_down.pt', 0.50)


def pose(img0, depth, box):

    depth, point_cloud = depth
    x,y,x1,y1  = box


    cropped = img0[int(y):int(y1), int(x):int(x1)]
    ob = up_classifier.detect_object(cropped)
    # TODO get bigger box

    for o in ob:
        c, _ = o

        # TODO opencv2
        #   - GET CENTER
        #   - GET YAW
        cx,cy = x+ ( (x1-x) // 2), y+ ((y1-y)//2)
        px,py,pz = pixel2cloud(point_cloud, cx,cy)

        return Pose(
            Point(trw(px,py,pz)),
            Rotation(0, 0, 0)
        )


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
        return ObjectDetectionResponse(False, [])
    else:
        # DETECT AND PUBLISH
        objects = []
        ob = classifier.detect_object(left)
        
        for o in ob:
            
            c, (x,y,x1,y1) = o
            rospy.loginfo("Found %d" % c)

            rotation = pose(left, (depth, point_cloud), (x,y,x1,y1))
          
            objects.append(
                Object(c, rotation)
            )
    return ObjectDetectionResponse(True, objects)
            

def init():
    rospy.loginfo("Loading models...")

  
    rospy.loginfo("Fused...")

    rospy.loginfo("Loading services...")
    rospy.Service('object_detection', ObjectDetection,object_detection)



