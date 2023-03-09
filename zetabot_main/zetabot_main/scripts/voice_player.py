#!/usr/bin/env python
import rospy
import actionlib

from threading import Thread
import pygame as pg
import os

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from zetabot_main.msg import MoveMsgs, MoveBaseActAction, MoveBaseActFeedback, MoveBaseActResult, MoveBaseActGoal
from actionlib_msgs.msg import GoalStatusArray

from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RosMaker():
    def __init__(self,type,topic_name,message_type):
        self._topic_name = topic_name
        self._message_type = message_type
        if type == "pub" :
            self._pub = rospy.Publisher(self._topic_name,self._message_type,queue_size=10)

        elif type == "sub" :
            self.msg = message_type()
            self._sub = rospy.Subscriber(self._topic_name,self._message_type,self.callback)

        elif type == "action" :
            self._action_name = topic_name

            message_type_str = str(message_type).split(".")[-1].split("'")[0]
            self.msg = globals()[message_type_str.replace("Action","Goal")]()
            self._feedback = globals()[message_type_str.replace("Action","Feedback")]()
            self._result = globals()[message_type_str.replace("Action","Result")]()

            self._as = actionlib.SimpleActionServer(self._action_name, message_type, execute_cb=self.execute_cb, auto_start = False)
            self._as.start()
        
        self.goal = MoveBaseActGoal()
        self.action_flag = False
        # battery_level_topic = "/battery_level"
        # battery_level_pub = rospy.Publisher(battery_level_topic,UInt8,queue_size=10)
 
    # instance method
    def callback(self,_msgs):
        self.msg = _msgs

    def publish(self,msgs):
        self._pub.publish(msgs)


    def execute_cb(self, goal):
        self.goal = goal
        return goal.result

class voice_player() :
    def __init__(self) :
        self.play_list = [] 
        self.player = RosMaker("sub","/voice_player",String)
        player_thread = Thread(target=self.make_list, args=(1, 100000))
        player_thread.start()
        pg.mixer.init(frequency=22050, size=-16, channels=2, buffer=4096)

    def player_controller(self) :
        while True:
            rospy.sleep(0.3)
            if len(self.play_list) != 0 :
                if not(pg.mixer.music.get_busy()):
                    pg.mixer.music.load(self.play_list.pop(0))
                    rospy.sleep(1)
                    pg.mixer.music.play()

    def make_list(self) :
        while True :
            rospy.sleep(0.3)
            if self.player.msg != None :
                play_sound = "~/voice/" + self.player.msg.data
                if not (play_sound in self.play_list) :
                    self.play_list.append(play_sound)
                self.player.msg = None