#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import Person
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('custom_chatter', Person)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = Person()
    msg.name = "ROS User"
    msg.age = 4

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
