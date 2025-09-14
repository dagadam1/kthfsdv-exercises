#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16

n = 4

def publisher():
    pub = rospy.Publisher('olsson', UInt16, queue_size=10)
    rospy.init_node('nodeA', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    k = n
    while not rospy.is_shutdown():
        rospy.loginfo(k)
        pub.publish(k)
        rate.sleep()
        k += n

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
