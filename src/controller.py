#!/usr/bin/env python
#
#


## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import Int16

def callback1(data):
	rospy.loginfo(rospy.get_caller_id() + 'FLEX: %s', data.data)


	
def callback2(data):
	rospy.loginfo(rospy.get_caller_id() + 'FSR: %s', data.data)


def controller():
    pub = rospy.Publisher('turn_servo', Int16, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("right_flex_value", Int16, callback1)
    rospy.Subscriber("right_fsr_value", Int16, callback2)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():    
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
