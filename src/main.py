#!/usr/bin/env python
"""

    This file is part of PiJoint.

    @package Vision
    @author: Alessandro Mizzaro
    @contact:
    @version: 1.0.0

"""
import service
import listener
import rospy

def main():
    """
    Main function
    """
    rospy.init_node('vision', anonymous=True)

    service.init()
    listener.init()
    rospy.spin()


if __name__ == '__main__':
    main()