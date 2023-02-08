#!/usr/bin/env python
from functools import partial
from threading import Lock

import cv2
import rospy

import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pijoint_vision.msg import RGBD
from pijoint_vision.msgs import Dictionary



def init():
    rospy.loginfo("Loading listner...")
    synchro = rospy.Publisher('/pjoint/_internal/camera', RGBD, queue_size=10)
    image_sub = message_filters.Subscriber('/ur5/zed_node/left/image_rect_color', Image)
    depth_sub = message_filters.Subscriber('/ur5/zed_node/depth/depth_registered', Image)
    ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)

    ts.registerCallback(lambda image, depth: synchro.publish(image, depth))


    
   
    

