
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


bridge = CvBridge()



def pose(up_classifier, img0, depth, box):
    (x,y,x1,y1)  = box
    cropped = img0[int(y):int(y1), int(x):int(x1)]

    ob = up_classifier.detect_object(cropped)
    # TODO get bigger box

    for o in ob:
        c, _ = o

        # TODO opencv2
        yaw = 0 
        pitch = 0
        roll = 0

        x = 0
        y = 0 
        z = 0

        return Pose(
            Point(x,y,z),
            Rotation(yaw, pitch, roll)
        )


def object_detection(classifiers, req):
    rospy.loginfo("Starting detection...")

    classifier, up_c = classifiers
    rgbd = rospy.wait_for_message('/pjoint/_internal/camera', RGBD)
    
    try:
        # Convert your ROS Image message to OpenCV2
        left = bridge.imgmsg_to_cv2(rgbd.image, 'bgr8')
        depth = bridge.imgmsg_to_cv2(rgbd.depth)
        point_cloud = rgbd.point_cloud
        cv2.resize(depth,(1920,1080))
        cv2.resize(left,(1920,1080))
        # point_cloud = rgbd.point_cloud

    except CvBridgeError as e:
        return ObjectDetectionResponse([])
    else:
        # DETECT AND PUBLISH
        objects = []
        ob = classifier.detect_object(left)
        
        for o in ob:
            
            c, (x,y,x1,y1) = o
            print('BOX', x,y,x1,y1)
            cx,cy = x+ ( (x1-x) // 2), y+ ((y1-y)//2)


            
            px,py,pz = pixel2cloud(point_cloud, cx,cy)
            print('POINTCLOUD: ', px, py, pz, 'CENTER', cx,cy, 'DEPTH', depth[cy,cx])
            print(sqrt(px**2 + pz**2 + py**2))

            rospy.loginfo("Found %d" % c)

            # rotation = pose(up_c, left, depth, (x,y,x1,y1))

          
            #objects.append(
            #    Object(c, rotation)
            #)
    return ObjectDetectionResponse(True, objects)
            

def init():
    rospy.loginfo("Loading models...")

    classifier = Model('src/best.pt', 0.30)
    up_classifier = Model('src/up_and_down.pt', 0.50)
    rospy.loginfo("Fused...")

    rospy.loginfo("Loading services...")
    rospy.Service('object_detection', ObjectDetection, partial(object_detection, (classifier, up_classifier)))



if __name__ == '__main__':
    rospy.wait_for_service('object_detection')
    service = rospy.ServiceProxy('object_detection', ObjectDetection)

    response =  service(True)


