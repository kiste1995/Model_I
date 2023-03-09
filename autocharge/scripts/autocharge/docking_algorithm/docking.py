from __future__ import print_function

import os
import sys

sys.path.append(os.path.abspath(os.path.dirname(__file__)))


import cv2
from time import time
from time import sleep

import rospy
from std_srvs.srv import Empty

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import threading

from recognition.recognizer import Recongnition
from log.logger import Logger

from deco import docking

# import queue
success = False
stranger = False


@docking
class DockingFunction:
    def __init__(self, ACP, ACS, ACSV, RCO):
        self.ACP = ACP
        self.ACS = ACS
        self.ACSV = ACSV
        self.RCO = RCO
        
        self.LOG = Logger()

        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)

        self.initParamets()

        # self.dp_queue = queue.Queue()

    def initParamets(self):
        self.sequence = "waiting"

        self.stop_time = 0
        self.stop_flag = False
        self.direction_flag = False
        self.search_double_check = False
        self.guidance_double_check = False

        self.search_fail_cnt = 0
        self.detect_fail_cnt = 0
        self.docking_fail_cnt = 0
        self.docking_cnt = 0

        self.bRunDockProc = False

    # def docking_main(self, cap):
    #     global success
    #     global stranger

    #     print("running docking processing")

    #     t = threading.Thread(target= self.docking_process)
    #     # t = threading.Thread(target= self.docking_process, args=((success, stranger),))
    #     t.daemon = True
    #     t.start()
        
    #     # dp_data = self.dp_queue.get()

    #     # return dp_data[0], dp_data[1]

    #     self.RCO.image_processing_finish_flag = False
    #     self.RCO.thd_end = False

    #     self.RCO.image_processing(cap)

    #     # while True:
    #     #     if self.bRunDockProc == False:
    #     #         break

    #     #     sleep(0.01)

    #     print("Exit docking processing")

    #     print("success:{}  stranger : {}".format(success, stranger))

    #     return success, stranger

        
    # def storeInQueue(self):
    #     def wrapper():
    #         self.dp_queue.put()
    #     return wrapper


    # @storeInQueue
    # def docking_process(self):
    def docking_main(self, cap):
        # global success
        # global stranger

        self.ACS.cancel_flag = False
        self.initParamets()

        self.RCO.image_processing_finish_flag = False
        self.RCO.thd_end = False

        t = threading.Thread(target= self.RCO.image_processing, args=(cap,))
        t.daemon = True
        t.start()
        print("image_processing")

        self.bRunDockProc = True

        while True:
            #print("main:", self.RCO.arr)
            
            if self.ACS.bEStop == True:
                print("EStop -->")
                
                if self.RCO.image_processing_finish_flag == False:
                    self.RCO.image_processing_finish_flag = True

                self.RCO.AllWindowsClose()

                self.docking_finish()
                
                self.bRunDockProc = False
                return True, True
            
            if self.ACS.cancel_flag:
                print("start auto parking cancel proc...")
                self.ACS.cancel_flag = False
                self.ACP.autocharge_pub(6)
                self.LOG.sequence_logger("autocharge_cancel")

                if self.RCO.image_processing_finish_flag == False:
                    self.RCO.image_processing_finish_flag = True

                self.RCO.AllWindowsClose()

                self.docking_finish()        

                sleep(10)

                cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: false\" -1 "
                os.system(cmdstr)     
                
                # success = False
                # stranger = False

                print("complete auto parking cancel proc...")

                self.bRunDockProc = False
                # return
                return False, False

            if self.stop_flag == True:
                self.stop_delay()
                self.stop_flag = False

            if self.sequence == "waiting":
                print("Sequence", self.sequence)
                self.waiting_sequence()

            elif self.sequence == "start":
                print("Sequence", self.sequence)
                self.start_sequence()

            elif self.sequence == "search":
                print("Sequence", self.sequence)
                s_double_check = self.search_sequence()
                if s_double_check:
                    # success = False
                    # stranger = s_double_check

                    self.bRunDockProc = False
                    # return
                    return False, s_double_check

            elif self.sequence == "adjustment":
                print("Sequence", self.sequence)
                self.adjustment_sequence()

            elif self.sequence == "guidance":
                print("Sequence", self.sequence)
                g_double_check = self.guidance_sequence()
                if g_double_check:
                    # success = False
                    # stranger = g_double_check
                    self.bRunDockProc = False
                    # return
                    return False, g_double_check

            elif self.sequence == "charging":
                print("Sequence", self.sequence)
                self.charging_sequence()

            elif self.sequence == "not_connected":
                self.not_connected_sequence()
                self.docking_finish()

            elif self.sequence == "finish":
                print("Sequence", self.sequence)
                self.finish_sequence()
                self.docking_finish()

            else:
                self.else_sequence()
                self.docking_finish()

            # if cv2.waitKey(1) == ord('q'):
            #     self.bRunDockProc = False
            #     break

            sleep(0.01)

    def waiting_sequence(self):
        self.ACP.autocharge_pub(1)

        self.LOG.sequence_logger("movebase_start")
        print('waiting sequence movebase client start')

        # self.clear_costmaps_srv()

        # sleep(2.0)

        print("charge pos x : " + str(self.charge_position_x) + " charge pos y : " + str(self.charge_position_y))

        self.movebase_client(self.charge_position_x, self.charge_position_y)

        if self.ACS.cancel_flag == True:
            print("Exit waiting sequence due to charging cancel")

            self.sequence = "cancel"
            # self.ACS.cancel_flag = False
            return

        self.ACP.breakturn_pub(False)
        self.ACSV.autocharge_start_turn(self.charge_degree)
        self.LOG.sequence_logger("movebase_finish")

        self.search_fail_cnt = 0
        self.detect_fail_cnt = 0
        self.docking_cnt = 0
        # self.docking_fail_cnt = 0
        self.sequence = "start"
        self.LOG.sequence_logger(self.sequence)

    def start_sequence(self):
        self.ACP.autocharge_pub(1)
        print("charging_station_state:", self.ACS.charging_station_state)

        if self.ACS.charging_station_state == "start":
            self.sequence = "search"
            self.LOG.sequence_logger(self.sequence)

        ###
        #self.sequence = "search"
        ###

    def search_sequence(self):
        self.ACP.autocharge_pub(2)

        if self.RCO.arr == []:
            if self.direction_flag == False: self.left_turn(0.06, 0, 0)
            elif self.direction_flag == True: self.right_turn(0.06, 0, 0)

        else:
            center_check = self.RCO.arr[0]

            if center_check == '-':
                if self.direction_flag == False: self.left_turn(0.06, 0, 0)
                elif self.direction_flag == True: self.right_turn(0.06, 0, 0)

            elif center_check == 'RIGHT':
                self.right_turn(0.02, 0, 0)

            elif center_check == 'LEFT':
                self.left_turn(0.02, 0, 0)

            elif center_check == 'CENTER':
                self.stop_flag = True
                sleep(0.3)
                if center_check == 'CENTER':
                    self.sequence = "adjustment"
                    self.LOG.sequence_logger(self.sequence)
                else: pass
            else: pass

            if self.search_fail_cnt < 2000:
                print("search_fail_cnt: ", self.search_fail_cnt)
                self.search_fail_cnt += 1
            elif self.search_fail_cnt >= 2000:
                self.stop_flag = True
                self.direction_flag = False
                self.sequence = "waiting"
                self.LOG.sequence_logger("search_fail")
                #docking_main restart
                self.search_double_check = True
                return self.search_double_check

    def adjustment_sequence(self):
        self.ACP.autocharge_pub(3)

        center_check = self.RCO.arr[0]
        robot_position = self.RCO.arr[1]
        degree = self.RCO.arr[2]
        target_distance = self.RCO.arr[3]

        if center_check == '-':
            self.stop_flag = True
            self.sequence = "search"

        else:
            if robot_position == 'LEFT':
                self.stop_flag = True
                self.sequence = "search"
                self.direction_flag = True
                if robot_position == 'LEFT':
                    self.left_turn(0.45, int(degree), target_distance)
            
            elif robot_position == 'RIGHT':
                self.stop_flag = True
                self.sequence = "search"
                self.direction_flag = False
                if robot_position == 'RIGHT':
                    self.right_turn(0.45, int(degree), target_distance)

            elif robot_position == 'CENTER':
                self.stop_flag = True
                self.sequence = "search"
                self.direction_flag = False
                if robot_position == 'CENTER':
                    self.ACP.initial_position_pub()
                    self.ACP.velocity_pub(True, -0.02, 0)
                    sleep(1)
                    self.sequence = "guidance"
                    self.LOG.sequence_logger(self.sequence)

    def guidance_sequence(self):
        self.ACP.autocharge_pub(4)

        center_check = self.RCO.arr[0]
        robot_position = self.RCO.arr[1]

        if self.ACS.sona_distance < 10:
            self.cancel_forward(0.05)
            self.direction_flag = False
            self.sequence = "waiting"
            self.LOG.sequence_logger("guidance_fail")

        elif self.ACS.sona_distance > 30:
            if robot_position == "CENTER":
                if self.ACS.sona_distance < 45:
                    if center_check == "LEFT": self.backward(0.01, 0.005)
                    elif center_check == "RIGHT": self.backward(0.01, -0.005)
                else:
                    self.backward(0.015, 0)

            elif robot_position == "LEFT":
                self.backward(0.015, -0.01)

            elif robot_position == "RIGHT":
                self.backward(0.015, 0.01)

            else:
                pass

                # if self.detect_fail_cnt < 10:
                #     print("detect_fail_cnt: ", self.detect_fail_cnt)
                #     self.detect_fail_cnt += 1
                # else:
                #     self.direction_flag = False
                #     self.sequence = "waiting"
                #     self.LOG.sequence_logger("detect_fail")
                #     #docking_main_restart
                #     self.guidance_double_check = True
                #     return self.guidance_double_check

        else:
            ###
            #self.RCO.image_processing_finish()
            ###
            
			# if self.docking_cnt < 300:            
            if self.docking_fail_cnt < 300:
                # print("docking_cnt: ", self.docking_cnt)
                print("docking_fail_cnt: ", self.docking_fail_cnt)
                self.docking_fail_cnt += 1
                # self.docking_cnt += 1
            else:
                self.cancel_forward(0.05)
                self.direction_flag = False
                self.sequence = "waiting"
                self.LOG.sequence_logger("docking_fail")

            if self.ACS.charging_station_state == "contact":
                self.stop_flag = True

                if self.RCO.image_processing_finish_flag == False:
                    self.RCO.image_processing_finish_flag = True

                self.ACP.led_control_pub("charging")
                self.sequence = "charging"
                self.LOG.sequence_logger(self.sequence)
                # ===================>
                self.RCO.bCharging = True

            elif self.ACS.charging_station_state == "left":
                self.backward(0.005, -0.01)
            elif self.ACS.charging_station_state == "right":
                self.backward(0.005, 0.01)
            else:
                self.backward(0.01, 0)

    def charging_sequence(self):
        self.ACP.autocharge_pub(5)
        self.stop_turn()

        print("SOC: ", self.ACS.battery_amount)
        self.RCO.thd_end = True
        
        if self.ACS.battery_amount > 95:
            self.sequence = "finish"
            self.LOG.sequence_logger(self.sequence)
        else:
            sleep(3)
            pass

    def finish_sequence(self):
        print("SOC: ", self.ACS.battery_amount)
        self.ACP.autocharge_pub(6)
        print("finish!!!")
        sleep(5)

        if self.ACS.battery_amount < 90:
            self.sequence = "charging"
            self.LOG.sequence_logger(self.sequence)

    def not_connected_sequence(self): #self.ACP.autocharge_pub(7)
        pass

    def else_sequence(self): #self.ACP.autocharge_pub(8)
        pass

    def docking_finish(self):
        self.stop_turn()


    def movebase_client(self, x, y):
        # self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        if self.ACS.cancel_flag == True:
            # self.ACS.cancel_flag = False

            print("inside movebase_client : auto parking cancel...")
            
            self.ACP.autocharge_pub(6)
            self.LOG.sequence_logger("autocharge_cancel")

            if self.RCO.image_processing_finish_flag == False:
                self.RCO.image_processing_finish_flag = True

            self.RCO.AllWindowsClose()

            self.docking_finish()             
            
            sleep(10)

            return

        # os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
        # os.system("rosservice call /move_base/clear_costmaps \"{}\"  ")
        

        # sleep(1.0)

        print("movebase_client start!")
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        print("goal setup! ==> x: %f  y:%f" %(x, y))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1
        # goal.target_pose.pose.orientation.z = -0.0053748554395
        # goal.target_pose.pose.orientation.w = 0.99998555536
                
        # self.clear_costmaps_srv()
        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
        
        sleep(1.0)
        
        self.ACS.goal_status = None
        print("wait_before: ", self.ACS.goal_status)
        
        print("send goal position!")
        client.send_goal(goal)
        print("wait goal position result!")
        wait = client.wait_for_result()

        print("wait_after: ", self.ACS.goal_status)

        if self.ACS.goal_status == 3:
            print("goal status result 3!")
        else:
            print("goal status result fail retry!")
            
            # print("move backward...")
            # self.backward(0.03, 0.0)
            # sleep(1)

            self.movebase_client(self.charge_position_x, self.charge_position_y)

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()

    def stop_delay(self):
        if self.stop_time < 10:
            print("stop_time: ", self.stop_time)
            self.stop_turn()
            self.stop_time += 1
            sleep(0.1)

        else:
            self.stop_time = 0

    def stop_turn(self):
        self.ACP.velocity_pub(True, 0, 0)

    def forward(self, velocity, target_distance):
        self.ACP.velocity_pub(True, velocity, 0)
        sleep_time = target_distance * 0.7
        sleep(int(sleep_time))

    def cancel_forward(self, velocity):
        self.ACP.velocity_pub(True, velocity, 0)
        sleep(3)

    def backward(self, velocity, angular):
        self.ACP.velocity_pub(True, -velocity, angular)

    def left_turn(self, velocity, _degree, target_distance):
        if _degree != 0:
            _target_degree = self.ACS.degree + 90

            if _target_degree <= 0:
                _target_degree = _target_degree + 360
            if _target_degree > 360:
                _target_degree = _target_degree - 360

            while True:
                self.ACP.velocity_pub(True, 0, velocity)

                _degree = self.ACS.degree

                #print("degree: ", _degree)
                #print("target_degree: ", _target_degree)

                if (_target_degree - 1) <= _degree <= (_target_degree + 1) :
                    self.stop_turn()
                    break

            print("first turn comple")

            sleep(0.3)
            self.forward(0.02, target_distance)
            self.stop_turn()
            sleep(0.3)

            _target_degree = self.ACS.degree - 65

            if _target_degree <= 0:
                _target_degree = _target_degree + 360
            if _target_degree > 360:
                _target_degree = _target_degree - 360

            while True:
                self.ACP.velocity_pub(True, 0, -velocity)

                _degree = self.ACS.degree

                #print("degree: ", _degree)
                #print("target_degree: ", _target_degree)

                if (_target_degree - 1) <= _degree <= (_target_degree + 1) :
                    self.stop_turn()
                    break

            sleep(1)
            print("second turn comple")
            
        else:
            self.ACP.velocity_pub(True, 0, velocity)

    def right_turn(self, velocity, _degree, target_distance):
        if _degree != 0:
            _target_degree = self.ACS.degree - 90

            if _target_degree <= 0:
                _target_degree = _target_degree + 360
            if _target_degree > 360:
                _target_degree = _target_degree - 360


            while True:
                self.ACP.velocity_pub(True, 0, -velocity)

                _degree = self.ACS.degree

                #print("degree: ", _degree)
                #print("target_degree: ", _target_degree)

                if (_target_degree - 1) <= self.ACS.degree <= (_target_degree + 1) :
                    self.stop_turn()
                    break

            print("first turn comple")
            sleep(0.3)
            self.forward(0.02, target_distance)
            self.stop_turn()
            sleep(0.3)

            _target_degree = self.ACS.degree + 65

            if _target_degree <= 0:
                _target_degree = _target_degree + 360
            if _target_degree > 360:
                _target_degree = _target_degree - 360

            while True:
                self.ACP.velocity_pub(True, 0, velocity)

                _degree = self.ACS.degree

                #print("degree: ", _degree)
                #print("target_degree: ", _target_degree)

                if (_target_degree - 1) <= self.ACS.degree <= (_target_degree + 1) :
                    self.stop_turn()
                    break

            sleep(1)
            print("second turn comple")

        else:
            self.ACP.velocity_pub(True, 0, -velocity)
