#!/usr/bin/env python

import rospy
from viam_event_data.msg import Event

NO_EVENT = 0

def publish_event():
    pub = rospy.Publisher('event', Event, queue_size=10)
    rospy.init_node('events', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        e = Event()
        e.event_id = NO_EVENT
        e.ts = rospy.get_time()
        rospy.loginfo(e)
        pub.publish(e)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_event()
    except rospy.ROSInterruptException:
        pass
