#!/usr/bin/env python
import rospy
import serial
import struct
from std_msgs.msg import String

port = "/dev/ttyACM3"
baud = 9600
ser = serial.Serial(port, baud)
curPos = [0,0,0,0,0,0]
goalPos = [1111,2222,3333,1111,2222,1111]
def tasks():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('armHandler', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)

        # ser.write('Q')
        # ser.flush()
        # data_raw = ser.readline()
        # data = data_raw.split(':')
        # if data[0] == 'A' and len(data) > 6:
        #     for x in range(6):
        #         curPos[x] = int(data[x+1])
        #         print(curPos[x])
        outputArray = bytearray()
        outputArray.append(65)
        for x in range(6):
            data = struct.pack(">H", goalPos[x])
            outputArray.append(data[0])
            outputArray.append(data[1])
        outputArray.append(95)
        ser.write(outputArray)

        # for x in range(len(curPos)):
        #     print(curPos[x])
        # print(len(curPos))
        rate.sleep()

if __name__ == '__main__':
    try:
        tasks()
    except rospy.ROSInterruptException:
        pass
    ser.close()
