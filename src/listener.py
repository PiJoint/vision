#!/usr/bin/env python
"""

    This file is part of PiJoint.

    @package Vision
    @author: Alessandro Mizzaro
    @contact:
    @version: 1.0.0

"""
from functools import partial
from threading import Lock

import cv2
import rospy

import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2


from pijoint_vision.msg import RGBD




def init():
    """
    Init function
    Time synchronizer
    """
    rospy.loginfo("Loading listner...")
    synchro = rospy.Publisher('/pjoint/_internal/camera', RGBD, queue_size=10)
    image_sub = message_filters.Subscriber('/ur5/zed_node/left/image_rect_color', Image)
    depth_sub = message_filters.Subscriber('/ur5/zed_node/depth/depth_registered', Image)
    point_sub = message_filters.Subscriber('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2)

    ts = message_filters.TimeSynchronizer([image_sub, depth_sub, point_sub], 10)

    ts.registerCallback(lambda image, depth, point_cloud: synchro.publish(image, depth, point_cloud))


    
   
    

