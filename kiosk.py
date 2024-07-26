#!/usr/bin/env python3
import rospy
import rospy as rp
import smach
from smach import *
from smach_ros import *

from actionlib import *
from actionlib_msgs.msg import *
from my_kiosk.msg import Order

from collections import deque

pub_topic_order = '/kiosk/order'
pub_topic_food = '/kiosk/food'
sub_topic_order = '/client/order'
sub_topic_food = '/chief/food'

waiting_list = deque()


def take_order():
    pub = rospy.Publisher(pub_topic_order, Order, queue_size=10)
    r = rospy.Rate(10.0)
    while len(waiting_list) == 0:
        rospy.sleep(0.1)
    order = Order()
    _id, _food, _beverage = waiting_list.popleft()
    order.id = _id
    order.food = _food
    order.beverage = _beverage
    rospy.loginfo("kiosk %s", str(_id))
    while not rospy.is_shutdown():
        pub.publish(order)
        rospy.loginfo("pub")
        r.sleep()


def delivery():
    pub = rospy.Publisher(sub_topic_food, Order)
    r = rospy.Rate(10.0)
    if waiting_list:
        order = Order()
        _id, _food, _beverage = waiting_list.popleft()
        order.id = _id
        order.food = _food
        order.beverage = _beverage
        while not rospy.is_shutdown():
            pub.publish(order)


"""
If this kiosk get new order from client,
This logic would be working. 
@subscribe: /client/order
@publish: /kiosk/order
"""


def order_cb(ud, msg):
    rospy.loginfo("[KIOSK] I took your order. (food: %s, beverage: %s)", str(msg.food), str(msg.beverage))
    if ud.id == msg.id:
        return True

    ud.id = msg.id
    ud.food = msg.food
    ud.beverage = msg.food
    waiting_list.append([msg.id, msg.food, msg.beverage])
    return False


"""
If this kiosk hear new food is ready from chief,
This logic would be working. 
@subscribe: /chief/food
@publish: /kiosk/food
"""


def food_cb(ud, msg):

    if ud.id != msg.id:
        rospy.loginfo("[KIOSK] Food is ready! (food: %s, beverage: %s)", str(msg.food), str(msg.beverage))
        return True

    return False


class Sender(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['id', 'food', 'beverage'], output_keys=[])
        self.pub = rospy.Publisher(pub_topic_order, Order, queue_size=10)
        self.r = rospy.Rate(10.0)

    def execute(self, ud):
        order = Order()
        order.id = ud.id
        order.food = ud.food
        order.beverage = ud.beverage
        rospy.loginfo("[KIOSK] I am gonna send this order to chief (id:%s)", str(ud.id))
        # while not rospy.is_shutdown():
        for i in range(200):
            self.pub.publish(order)
            self.r.sleep()
        return 'done'


class Delivery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['id', 'food', 'beverage'], output_keys=[])
        self.pub = rospy.Publisher(pub_topic_food, Order, queue_size=10)
        self.r = rospy.Rate(10.0)

    def execute(self, ud):
        order = Order()
        order.id = ud.id
        order.food = ud.food
        order.beverage = ud.beverage
        rospy.loginfo("[KIOSK] Delivery Service now. Arrived soon (id: %s)", str(ud.id))
        # while not rospy.is_shutdown():
        for i in range(200):
            self.pub.publish(order)
            # rospy.loginfo("pub")
            self.r.sleep()
        return 'done'


def kiosk():
    rospy.init_node('Kiosk')

    sm = smach.StateMachine(['valid', 'invalid', 'preempted'])
    sm.set_initial_state(['WAIT_ORDER'])
    sm.userdata.id = 0
    sm.userdata.food = []
    sm.userdata.beverage = []
    with sm:
        smach.StateMachine.add(
            'WAIT_ORDER',
            MonitorState(sub_topic_order, Order, order_cb, input_keys=['id'], output_keys=['id', 'food', 'beverage']),
            transitions={'invalid': 'SEND_ORDER'}
        )
        smach.StateMachine.add(
            'SEND_ORDER', Sender(), transitions={'done': 'WAIT_FOOD'}
        )
        smach.StateMachine.add(
            'WAIT_FOOD',
            MonitorState(sub_topic_food, Order, food_cb,
                         input_keys=['id'], output_keys=['id', 'food', 'beverage']),
            transitions={'invalid': 'DELIVERY'}
        )
        smach.StateMachine.add(
            'DELIVERY', Delivery(), transitions={'done': 'WAIT_ORDER'}
        )

    sis = IntrospectionServer('Kiosk', sm, '/KIOSK')
    sis.start()

    outcome = sm.execute()
    rospy.loginfo("[KIOSK] status: %s", outcome)
    # rospy.loginfo("[KIOSK] Order Number %s is fully completed. Happy Meal!", 100)

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    kiosk()
