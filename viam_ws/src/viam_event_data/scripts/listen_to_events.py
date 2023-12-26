#!/usr/bin/env python

import rospy
from viam_event_data.msg import Event

"""
a simple set of events we will use for testing
Three events as well as how the events are set and cleared
"""
NO_EVENT = 0
SIMPLE_EVENT_ON = 1
SIMPLE_EVENT_OFF = 2
GENERIC_EVENT_ON = 3
GENERIC_EVENT_OFF = 4
SOME_EVENT_ON = 5
SOME_EVENT_OFF = 6

events = []

def add_event(eid):
    global events
    if eid not in events:
        events.append(eid)
        return True
    rospy.loginfo('event already exists')
    return False

def remove_event(eid):
    global events
    if eid not in events:
        rospy.loginfo('event not found')
        return False
    events.remove(eid)
    return True
    
def callback(data):
    global events
    if data.event_id == NO_EVENT:
        rospy.loginfo('no event')
    elif data.event_id == SIMPLE_EVENT_ON:
        add_event(SIMPLE_EVENT_ON)
        rospy.loginfo('simple event start')
    elif data.event_id == GENERIC_EVENT_ON:
        add_event(GENERIC_EVENT_ON)
        rospy.loginfo('generic event started')
    elif data.event_id == SOME_EVENT_ON:
        add_event(SOME_EVENT_ON)
        rospy.loginfo('some event started')
    elif data.event_id == SIMPLE_EVENT_OFF:
        remove_event(SIMPLE_EVENT_ON)
        rospy.loginfo('simple event cleared')
    elif data.event_id == GENERIC_EVENT_OFF:
        remove_event(GENERIC_EVENT_ON)
        rospy.loginfo('generic event cleared')
    elif data.event_id == SOME_EVENT_OFF:
        remove_event(SOME_EVENT_ON)
        rospy.loginfo('some event cleared')

    if len(events) > 0:
        rospy.loginfo(f'we have events occurring: {events}')
    else:
        rospy.loginfo('no events occurring')

def listener():
    rospy.init_node('event_listener', anonymous=True)
    rospy.Subscriber('event', Event, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
