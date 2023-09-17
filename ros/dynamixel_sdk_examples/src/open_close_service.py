#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse
#from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

class drone_catch():
    def __init__(self):
        print("service ready")
        self.control = rospy.Publisher("/sync_set_position", SyncSetPosition, queue_size=1)
        release_drone = rospy.Service("/release", Trigger, self.release)
        close = rospy.Service("/close", Trigger, self.close)

    def release(self, req):
            res = TriggerResponse()
            try:
                rel_msg = SyncSetPosition()
                rel_msg.id1 = 2
                rel_msg.id2 = 4
                rel_msg.position1 = 1000
                rel_msg.position2 = 1000
                self.control.publish(rel_msg)
                res.success = True
            except (rospy.ServiceException, rospy.ROSException) as e:
                res.success = False
                print("Service call failed: %s"%e)

            return res
    
    def close(self, req):
            res = TriggerResponse()
            try:
                close_msg = SyncSetPosition()
                close_msg.id1 = 2
                close_msg.id2 = 4
                close_msg.position1 = 2035
                close_msg.position2 = 2035
                self.control.publish(close_msg)
                res.success = True
            except (rospy.ServiceException, rospy.ROSException) as e:
                res.success = False
                print("Service call failed: %s"%e)

            return res

if __name__ == '__main__':
    rospy.init_node('motor_control')
    test = drone_catch()
    rospy.spin()