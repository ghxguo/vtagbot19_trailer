#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String

port = "/dev/ttyACM0"
baud = 115200
ser = serial.Serial(port, baud)

def tasks():
    # pub = rospy.Publisher('chatter', String, queue_size=10)

    rospy.init_node('armHandler', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        ser.write('A')
        rate.sleep()

if __name__ == '__main__':
    try:
        tasks())
    except rospy.ROSInterruptException:
        pass
    ser.close()
