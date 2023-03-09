#!/usr/bin/env python
import rospy
import os

from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from zetabot_main.srv import ModuleControllerSrv

class led_controller(object) :
    def __init__(self) :
        self.led_pub = RosMaker("pub","/led_command",Int64)

    def led_controll(self,f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue) :
        command = 0
        for i in [f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue] : 
            command = (command << 8) | i
        self.led_pub.publish(command)
        print(hex(command))
        

class RosMaker(object):
    def __init__(self,type,topic_name,message_type):
        self._topic_name = topic_name
        self._message_type = message_type
        if type == "pub" :
            self._pub = rospy.Publisher(self._topic_name,self._message_type,queue_size=10)

        elif type == "sub" :
            self.msg = message_type()
            self._sub = rospy.Subscriber(self._topic_name,self._message_type,self.callback)
        
        # battery_level_topic = "/battery_level"
        # battery_level_pub = rospy.Publisher(battery_level_topic,UInt8,queue_size=10)
 
    # instance method
    def callback(self,_msgs):
        self.msg = _msgs

    def publish(self,msgs):
        self._pub.publish(msgs)