from cv_bridge import CvBridge, CvBridgeError

import rospy
from pijoint_vision.srv import ObjectDetection
import cv2

bridge = CvBridge()
client = rospy.ServiceProxy('object_detection', ObjectDetection)

res = client(True)

for ob in res.objects:
    print("DETECTED")
    print("\t","Class: ", ob.o_class)
    print("\t","Ground: ", ob.box.ground)
    print("\t","Center: ")
    print("\t\t","X: ", ob.box.center.x)
    print("\t\t","Y: ", ob.box.center.y)
    print("\t\t","Z: ", ob.box.center.z)
    print("\t","Piolini: ")
    print("\t\t","X: ", ob.box.piolini.x)
    print("\t\t","Y: ", ob.box.piolini.y)
    print("\t\t","Z: ", ob.box.piolini.z)
    print("\t","Rotation: ")
    print("\t\t","Yaw: ", ob.box.yaw)

    # cv2.imshow("Image", bridge.imgmsg_to_cv2(ob.raw))
    cv2.imshow("Yolov7", bridge.imgmsg_to_cv2(ob.yolo))
    cv2.waitKey(0)


