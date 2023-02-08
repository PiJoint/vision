import service
import listener
import rospy

def main():
    rospy.init_node('vision', anonymous=True)

    service.init()
    listener.init()
    rospy.spin()


if __name__ == '__main__':
    main()