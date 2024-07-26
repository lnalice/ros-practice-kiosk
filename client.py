#!/usr/bin/env python3
import random

import rospy
import smach
from smach import *
from smach_ros import *

from actionlib import *
from actionlib_msgs.msg import *

from my_kiosk.msg import Order

import argparse
import os

pub_topic = '/client/order'
sub_topic = '/kiosk/food'
"""
Client orders new menu (food, beverage, client_id(pid))
@subscribe: /kiosk/food
@publish: /client/order
"""
class Sender(smach.State):
    def __init__(self,menu):
        smach.State.__init__(self, outcomes=['done'], input_keys=['id', 'food', 'beverage'], output_keys=[])
        self.pub = rospy.Publisher(pub_topic, Order, queue_size=10)
        self.r = rospy.Rate(10.0)
        self.menu = menu
    def execute(self,ud):

        order = Order()
        order.id = os.getpid()
        order.food = self.menu.food
        order.beverage = self.menu.beverage
        rospy.loginfo("[CLIENT] pid: %s, food: %s, beverage: %s", str(os.getpid()), str(self.menu.food),str(self.menu.beverage))
        # while not rospy.is_shutdown():
        rospy.loginfo("[CLIENT] publishing... ")
        for i in range(200):
            self.pub.publish(order)

            self.r.sleep()
        return 'done'

def food_cb(ud, msg):
    if 'id' in ud:
        ud.id = msg.id
        ud.food = msg.food
        ud.beverage = msg.beverage
        return False
    return True

class Eating(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['id', 'food', 'beverage'])
    def execute(self,ud):
        for i in range(3):
            rospy.loginfo("[CLIENT]Client have a meal now...")
            rospy.loginfo("id: %s, food: %s, beverage: %s\n\n", str(ud.id), str(ud.food), str(ud.beverage))
            rospy.sleep(3)
        return 'done'


def client(menu):
    rospy.init_node('Client')

    sm = StateMachine(['valid', 'invalid', 'preempted', 'done'])
    sm.set_initial_state(['ORDER'])

    sm.userdata.id = 0
    sm.userdata.food = []
    sm.userdata.beverage = []

    with sm:
        smach.StateMachine.add(
            'ORDER', Sender(menu), transitions={'done': 'WAIT_FOOD'}
        )
        StateMachine.add(
            'WAIT_FOOD',
            MonitorState(sub_topic, Order, food_cb,
                         input_keys=['id'], output_keys=['id','food','beverage']),
            transitions={'invalid': 'EAT'}
        )
        StateMachine.add(
            'EAT', Eating(), transitions={'done': 'WAIT_FOOD'}
        )
        outcome = sm.execute()


parser = argparse.ArgumentParser(description="Ok Your order",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '-f', '--food', required=True, nargs='+', help='Food Menu')
parser.add_argument(
    '-b', '--beverage', required=True, nargs='+', help='Beverage Menu')
args = parser.parse_args()

if __name__ == "__main__":
    client(menu=args)
