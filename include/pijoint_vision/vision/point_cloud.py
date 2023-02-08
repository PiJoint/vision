import sys

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point


import numpy as np

def todtype7(msg):
    dtype = np.dtype(np.float32)
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    img = np.ndarray(shape=(msg.row_step * msg.height, 1),
                           dtype=dtype, buffer=msg.data)
    if msg.is_bigendian == (sys.byteorder == 'little'):
        img = img.byteswap().newbyteorder()
    return img


def read_float(msg, start, len=12):
    a = msg.data

    dtype = np.dtype(np.float32)
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    return np.ndarray(shape=(1,3),
                           dtype=dtype, buffer=msg.data[start: start+len])[0]


    


def pixel2cloud(point_cloud: PointCloud2, x: int, y: int):
    print(point_cloud.point_step)
    

    w = point_cloud.width
    h = point_cloud.height
    
    array_position = y * point_cloud.row_step + x * point_cloud.point_step


    data = read_float(point_cloud, array_position)
    print(data)
    #x = array_position + point_cloud.fields[0].offset
    #y = array_position + point_cloud.fields[1].offset
    #z = array_position + point_cloud.fields[2].offset

    return \
        data[0], data[1], data[2]
    