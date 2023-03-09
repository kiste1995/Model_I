#!/usr/bin/env python
from __future__ import print_function

import rospy
import threading
from time import time
from time import sleep

from std_srvs.srv import Empty

import actionlib
from zetabot_main.msg import ChargingAction,ChargingActionGoal,ChargingFeedback,ChargingActionResult

from autocharge_ros.autocharge_pub import AutoChargePublisher
from autocharge_ros.autocharge_sub import AutoChargeSubscriber
from autocharge_ros.autocharge_srv import AutoChargeSurvice
from docking_algorithm.docking import DockingFunction
from recognition.recognizer import Recongnition

import os, sys
import re
import cv2

ACP = AutoChargePublisher()
ACS = AutoChargeSubscriber()
ACSV = AutoChargeSurvice()
RCO = Recongnition()

class chargingAction(object):
    print("chargingAction start!!!")

    _result = ChargingActionResult()

    def __init__(self, name):
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)

        vindex = self.video_detect_index()
        # print("video device index : ")
        # print(vindex)

        self.cap = cv2.VideoCapture(vindex)

        print("detect video index:" + str(vindex))

        if not self.cap.isOpened():
            print("camera open failed! Exit ros package...")

            sys.exit()
            
        else:
            print("width:{}, height: {}".format(self.cap.get(3), self.cap.get(4)))

            self._action_name = name
            self._as = actionlib.SimpleActionServer(self._action_name, ChargingAction, execute_cb=self.execute_cb, auto_start = False)
            self._as.start()

        self.dock_Func = DockingFunction(ACP, ACS, ACSV, RCO)

    def execute_cb(self, goal):

        print("autocharge_main start!!!")

        while True:

            success = True
            stranger = False
            # self.success = True
            # self.stranger = False
            
            # dock_Func.docking_main(self.cap, self.success, self.stranger)
            success, stranger = self.dock_Func.docking_main(self.cap)
            # dock_Func = None

            self.clear_costmaps_srv()
            
            if success == True and stranger == False:
            # if self.success == True and self.stranger == False:
                #os.system("mplayer ~/voice/charging_done.mp3")
                self._result.result = "end_autocharge_mode"
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)
                break
            
            elif success == False and stranger == False:
            # elif self.success == False and self.stranger == False:                
                #os.system("mplayer ~/voice/charging_cancel.mp3")
                self._result.result = "charging_cancle_mode"
                ACP.led_control_pub("air_condition")
                rospy.loginfo('%s: cancled' % self._action_name)
                self._as.set_succeeded(self._result)
                print("Cancel : autocharging_end!!!!!!!!!!!!!!")
                break
            
            elif success == False and stranger == True:
            # elif self.success == False and self.stranger == True:
                self._result.result = "charging_restart_mode"
                rospy.loginfo('%s: docking restart' % self._action_name)
                # self._as.set_succeeded(self._result)
                continue
            
            elif success == True and stranger == True:
                self._result.result = "charging_estop_mode"
                rospy.loginfo('%s: estop' % self._action_name)

            sleep(0.01)

        print("autocharge_main stop!!!")


    def video_detect_index(self):
        video_idx = None

        for video_idx_file in os.listdir("/sys/class/video4linux"):
            video_file_path = os.path.realpath("/sys/class/video4linux/" + video_idx_file)

            for video_files in os.listdir(video_file_path):
                if 'name' in video_files:
                    name = open(os.path.realpath(video_file_path + '/' + 'name'), 'r').readline()
                    print("name:" + name)

                    if re.match('KINGSEN', name) != None:
                        index = open(os.path.realpath(video_file_path + '/' + 'index'), 'r').readline()
                        print("index:")
                        print(index)
                        if re.match('0', index) != None:
                            video_idx = "/dev/" + video_idx_file
                            print(video_idx)


        return video_idx


if __name__ == '__main__':
    rospy.init_node('charging_server_c')

    server = chargingAction("charging_act")

    # dock_Func = DockingFunction(ACP, ACS, ACSV, RCO)
    # dock_Func.docking_main(self.cap)

    rospy.spin()

    if server.cap.isOpened():
        server.cap.release()

"""
if __name__ == '__main__':
    recog = Recongnition()

    while True:
        
        recog.image_processing()
        print("FPS: %f, Delay: %dms" %(recog.fps, recog.delay))

        if cv2.waitKey(recog.delay + 3) == ord('q'):
            break

    recog.finish()
"""
