#!/usr/bin/env python3
import rospy
import rospy as rp
import smach
from smach import *
from smach_ros import *

from actionlib import *
from actionlib_msgs.msg import *
from my_kiosk.msg import Order

"""
If this kiosk get new order from client,
This logic would be working. 
@subscribe: /client/order
@publish: /kiosk/order
"""
class NewOrder(smach.State):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['new_order', 'empty'],
                                    input_keys=['foo'])
        self.topic_client_order = "/client/order"
        self.topic_kiosk_order = "/kiosk/order"
        self.new_order = False
        self.sub = rospy.Subscriber(self.topic_client_order, Order, self.send_to_chief, queue_size=10)
        self.pub = rospy.Publisher(self.topic_kiosk_order, Order, queue_size=10)

    def send_to_chief(self, msg):
        rospy.loginfo("[Kiosk] New order is arrived now!")
        rospy.loginfo("[Kiosk] food: %s, beverage: %s (client id: %s)",
                      str(msg.food), str(msg.beverage), str(msg.id))

        self.pub.publish(msg)
        rospy.loginfo("[Kiosk] New order is submitted to chief!")

        self.new_order = True

    def execute(self, ud):
        rospy.loginfo("Kiosk is working")
        if self.new_order:
            self.new_order = False
            return "new_order"
        else:
            return "empty"


"""
If this kiosk hear new food is ready from chief,
This logic would be working. 
@subscribe: /chief/food
@publish: /kiosk/food
"""
class FoodDelivery(smach.State):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['new_food', 'empty'])
        self.topic_chief_food = "/chief/food"
        self.topic_kiosk_food = "/kiosk/food"
        self.food_ready = False

        self.sub = rospy.Subscriber(self.topic_chief_food, Order, self.delivery_to_client, queue_size=10)
        self.pub = rospy.Publisher(self.topic_kiosk_food, Order, queue_size=10)

    def send_to_chief(self, msg):
        rospy.loginfo("[Kiosk] New food is already ready")
        rospy.loginfo("[Kiosk] food: %s, beverage: %s (client id: %s)",
                      str(msg.food), str(msg.beverage), str(msg.id))

        self.pub.publish(msg)
        rospy.loginfo("[Kiosk] New food is delivered to client now!")

        self.food_ready = True

    def execute(self, ud):
        rospy.loginfo("Kiosk is working")
        if self.food_ready:
            self.food_ready = False
            return "new_food"
        else:
            return "empty"


class Kiosk(smach.state):
    def __init__(self):
        self.i = 0

    def run(self):
        rospy.init_node('Kiosk')
        pubsub = smach.Concurrence(outcomes=['order_completed', 'order_error'])
        with pubsub:
            smach.Concurrence.add("NEW_ORDER", NewOrder())
            smach.Concurrence.add("FOOD_DELIVERY", FoodDelivery())

        sis = IntrospectionServer('kiosk', pubsub, '/KIOSK')
        sis.start()

        outcome = pubsub.execute()
        rospy.loginfo("Order Number %s is fully completed. Happy Meal!", self.i)

        rospy.spin()
        sis.stop()


if __name__ == "__main__":
    Kiosk.run()
