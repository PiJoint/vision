

from pijoint_vision.srv import ObjectDetection,ObjectDetectionResponse
from pijoint_vision.msg import RGBD, Object,Pose,Rotation
from geometry_msgs.msg import Point
import rospy

from cv_bridge import CvBridge, CvBridgeError
from pijoint_vision.ai import Model
from pijoint_vision.vision import pixel2cloud


bridge = CvBridge()

rospy.loginfo("Loading models...")

classifier = Model('src/best.pt', 0.30)
up_classifier = Model('src/up_and_down.pt', 0.50)
rospy.loginfo("Fused...")

def pose(img0, depth, box):
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


def object_detection(req):
    rospy.loginfo("Starting detection...")
    rgbd = rospy.wait_for_message('/pjoint/_internal/camera', RGBD)
    
    try:
        # Convert your ROS Image message to OpenCV2
        left = bridge.imgmsg_to_cv2(rgbd.image, 'bgr8')
        depth = bridge.imgmsg_to_cv2(rgbd.depth)

    except CvBridgeError as e:
        return ObjectDetectionResponse([])
    else:
        # DETECT AND PUBLISH
        objects = []
        ob = classifier.detect_object(left)
        
        for o in ob:
            
            c, (x,y,x1,y1) = o
            rospy.loginfo("Found %d" % c)

            rotation = pose(left, depth, (x,y,x1,y1))

            objects.append(
                Object(c, rotation)
            )
    return ObjectDetectionResponse(objects)
            

def init():
  rospy.loginfo("Loading services...")
  rospy.Service('object_detection', ObjectDetection, object_detection)



