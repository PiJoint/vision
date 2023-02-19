#!/usr/bin/env python
"""!

    This file is part of PiJoint.

    @package pijoint_vision vision
    @author: Alessandro Mizzaro
    @version: 1.0.0
"""
import sys

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point


import numpy as np


def read_floats(msg, start, len=12):
    """!
    Implementations of point_cloud2.read_points
    """
    a = msg.data

    dtype = np.dtype(np.float32)
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    if msg.is_bigendian == (sys.byteorder == 'little'):
            im = im.byteswap().newbyteorder()
    return np.ndarray(shape=(1,3),
                           dtype=dtype, buffer=msg.data[start: start+len])[0]


    


def pixel2cloud(point_cloud: PointCloud2, x: int, y: int):
    """!
    Implementations of point_cloud2.read_points
    """
    array_position = y * point_cloud.row_step + x * point_cloud.point_step
    data = read_floats(point_cloud, array_position)

    return \
        data[0], data[1], data[2]
    