#!/usr/bin/env python
import rospy
import serial
import struct
from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray

port = "/dev/ttyACM0"
baud = 9600
ser = serial.Serial(port, baud)
curPos = [0,0,0,0,0,0]
goalPos = [0,0,0,0,0,0]

def goalPos_cb(data):
    global goalPos
    for x in range(6):
        goalPos[x] = data.data[x]
def tasks():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    armCurPos_pub = rospy.Publisher('armCurPos', UInt16MultiArray, queue_size = 10)
    rospy.init_node('armHandler', anonymous=True)
    rospy.Subscriber("armGoalPos", UInt16MultiArray, goalPos_cb)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)

        ser.write('Q')
        ser.flush()
        data_raw = ser.readline()
        data = data_raw.split(':')
        curPos = [0,0,0,0,0,0]
        if data[0] == 'A' and len(data) > 6:
            for x in range(6):
                curPos[x] = int(data[x+1])
        armCurPos_pub.publish(data=curPos)
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
