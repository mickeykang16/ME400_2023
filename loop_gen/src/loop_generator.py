#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('loop_gen', Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():        
        pub.publish(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
