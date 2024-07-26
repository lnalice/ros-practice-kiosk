#!/usr/bin/env python3
from collections import deque

import rospy
import smach
from smach import *
from smach_ros import *

from actionlib import *
from actionlib_msgs.msg import *

from my_kiosk.msg import Order

import random
import argparse
import os
waiting_list = deque()

"""
Chief made food for client
@subscribe: /kiosk/order
@publish: /chief/food
"""

pub_topic= '/chief/food'


def food_cb(ud, msg):
    waiting_list.append([msg.id, msg.food, msg.beverage])
    if 'id' in ud:
        ud.id = msg.id
        ud.food = msg.food
        ud.beverage = msg.beverage
        return False
    return True

class Cooking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['id', 'food', 'beverage'])
    def execute(self,ud):

        for i in range(3):
            rospy.loginfo("[CHIEF] chief is cooking now...")
            rospy.loginfo("[CHIEF] id: %s, food: %s, beverage: %s\n\n", str(ud.id), str(ud.food), str(ud.beverage))
            rospy.sleep(3)
        return 'done'

class Sender(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['id', 'food', 'beverage'], output_keys=[])
        self.pub = rospy.Publisher(pub_topic, Order, queue_size=10)
        self.r = rospy.Rate(10.0)
    def execute(self,ud):

        order = Order()
        order.id = ud.id
        order.food = ud.food
        order.beverage = ud.beverage
        rospy.loginfo("[CHIEF] the food for %s is ready!", str(ud.id))
        # while not rospy.is_shutdown():
        rospy.loginfo("[CHIEF] publishing... ")
        for i in range(200):
            self.pub.publish(order)

            self.r.sleep()
        return 'done'


def chief():
    rospy.init_node('Chief')

    sm = smach.StateMachine(outcomes=['food_completed','valid', 'invalid', 'preempted'])
    # sm = StateMachine(['valid', 'invalid', 'preempted'])
    sm.set_initial_state(['WAIT_ORDER'])

    sm.userdata.id = 0
    sm.userdata.food = []
    sm.userdata.beverage = []

    with sm:
        smach.StateMachine.add(
            'WAIT_ORDER',
            MonitorState('/kiosk/order', Order, food_cb, input_keys=['id', 'food', 'beverage'],
                         output_keys=['id', 'food', 'beverage']), transitions={'invalid': 'COOKING'})
        smach.StateMachine.add(
            'COOKING', Cooking(), transitions={'done': 'HAND_OVER'}
        )
        smach.StateMachine.add(
            'HAND_OVER', Sender(), transitions={'done': 'WAIT_ORDER'}
        )

    sis = IntrospectionServer('Chief', sm, '/CHIEF')
    sis.start()

    outcome = sm.execute()
    rospy.loginfo("[CHIEF] status: %s", outcome)

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    chief()
