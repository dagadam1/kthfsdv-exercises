#!/usr/bin/env python

q = 0.15

import rospy
from std_msgs.msg import UInt16, Float32

class NodeB:
    def __init__(self):
        rospy.init_node('nodeB', anonymous=True)
        self.sub = rospy.Subscriber('olsson', UInt16, self.callback)
        self.pub = rospy.Publisher('/kthfs/result', Float32, queue_size=10)
        
    def callback(self, data):
        result = data.data / q
        rospy.loginfo('Result: %s', result)
        self.pub.publish(result)

def nodeB():
    node = NodeB()
    rospy.spin()

if __name__ == '__main__':
    nodeB()
