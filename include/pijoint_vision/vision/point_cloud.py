from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point



def pixel2cloud(point_cloud: PointCloud2, x: int, y: int):
    pass

    w = point_cloud.width
    h = point_cloud.height

    array_position = y * point_cloud.row_step + x * point_cloud.point_step


    x = array_position + point_cloud.fields[0].offset
    y = array_position + point_cloud.fields[1].offset
    z = array_position + point_cloud.fields[2].offset

    return Point(
        point_cloud.data[x], point_cloud.data[y], point_cloud[z]
    )