#!/usr/bin/env python

import os, sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
from python_qt_binding import loadUi
import time
import yaml
import rospy
from zetabank_msgs.msg import NavigationControl, NavigationControlStatus
import os
import numpy as np
import re
import math

from zetabank_msgs.msg import ZBDSRSetTraj, ZBDSRTrajStatus
from autocharge.msg import ChargingAction, ChargingActionGoal, ChargingGoal
import actionlib
import threading
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import std_msgs
from std_msgs.msg import Bool, String, UInt8

from zetabot_main.msg import PowerControlMsgs
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty

rospy.init_node('naviGUI_node')

navirviz_fname = "/home/zetabank/catkin_ws/src/zetabank_robot_control/zetabank_navigation/rviz/zetabank_nav.rviz"
slamrviz_fname = "/home/zetabank/catkin_ws/src/zetabank_robot_control/zetabank_slam/rviz/zetabank_slam.rviz"

rootdir = rospy.get_param('~root_dir')
fname = rootdir + "/scripts/ncwp_gui.ui"
# yaml_fname = rootdir +"/config/HDIWP7.yaml"
# yaml_fname = rootdir +"/config/HDIWP9.yaml"
# yaml_fname = rootdir +"/config/HDIWP10.yaml"
# yaml_fname = rootdir +"/config/HDIWP11.yaml" #Hustar_office_HollowDI_Test
# yaml_fname = rootdir +"/config/HDIWP12.yaml" #Conf_pose
# yaml_fname = rootdir +"/config/HDIWP13.yaml" #Map_resolution(0.01)
yaml_fname = rootdir +"/config/HDIWP55.yaml" #Hallow_pose (3Box)

form_class = uic.loadUiType(fname)[0]

ICON_RED_LED = ":/icons/red-led-on.png"
ICON_GRAY_LED = ":/icons/gray-led-on.png"
ICON_BLUE_LED = ":/icons/blue-led-on.png"
ICON_GREEN_LED = ":/icons/green-led-on.png"
ICON_ORANGE_LED = ":/icons/orange-led-on.png"
ICON_YELLOW_LED = ":/icons/yellow-led-on.png"

moveBindings = {
        'w':(1,0),
        'a':(0,1),
        'd':(0,-1),
        'x':(-1,0),
        }

speedBindings={
        'u':(1.1,1.1),
        'm':(0.9,0.9),
        'i':(1.1,1.0),
        ',':(0.9,1.0),
        'o':(1.0,1.1),
        '.':(1.0,0.9),
        }

TeleOP_Cont = True        

convkey = 0
prev_convkey = 0
convkey_val = 0
bRunMCtrl = False

DTRunTime = 10

bHDIWPFileLoadOK = False
bCRWPFileLoadOK = False

initSpeed = 0.1
initTurn = 0.15

MATH_RAD2DEG = 57.2957795

def vels(target_linear_vel, target_angular_vel):
    return "currently ==> linear vel:%s angular vel:%s" % (target_linear_vel,target_angular_vel)


class Thread_AutoParkingCharingStation(QThread):
    navi_runstat = pyqtSignal(str)
    
    def __init__(self, parent = None):
        super(Thread_AutoParkingCharingStation, self).__init__(parent)
        self.working = True
        
        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)

        rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)

        self.naviStatus = NavigationControlStatus.IDLING
        self.prevnaviStatus = NavigationControlStatus.IDLING

        self.RunType = 0
        

    def CallbackRunNaviCtrlStatus(self, request):
        self.naviStatus = request.status

        if(self.prevnaviStatus != self.naviStatus):
            mstr = request.status_description
            print(mstr)
            
            self.navi_runstat.emit(mstr)

            self.prevnaviStatus = self.naviStatus

    def setRunType(self, type):
        self.RunType = type
            
    def gotoWayPoint(self, wpname):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = wpname
        self.ncpub.publish(nc)
              
        mstr = "[WP] Return to charging station way point : " + nc.goal_name
        print(mstr)
        
    def run(self):

        if type == 1:
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("chargingpos")

            sleep(5)

            runcnt = 0
            
            while self.bRunFlag:
                if self.bRunFlag == False:
                    self.StopWaypoint()

                    self.navi_runstat.emit("Stop chargingpos DI process...")
                    
                    sleep(1)
                    
                    self.stop()
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause chargingpos DI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart chargingpos DI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    
                    print("Arrived at chargingpos...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt < self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("chargingpos")

                        self.navi_runstat.emit("Restart chargingpos")

                    else:
                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) chargingpos")

                    break

                sleep(0.1)

            print("Arrived at charging station position...")  


            client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            client.wait_for_server()

            goal = ChargingGoal()
            client.send_goal(goal)

            print("Parking charging station...")

            while self.bRunFlag:
                if self.bRunFlag == False:
                    cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
                    os.system(cmdstr)

                    self.navi_runstat.emit("Stop auto charging process...")
                    
                    sleep(1)
                    
                    self.stop()
                    
                if self.csStatus == "contact":
                    print("====>csStatus : " + self.csStatus)
                    self.navi_runstat.emit("Complete parking charging station.")
                    break
                    
                sleep(0.1)

            else:
                cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
                os.system(cmdstr)

                self.navi_runstat.emit("Cencel charging process...")
            
            
class Thread_ConfRoomDisfection(QThread):
    navi_runstat = pyqtSignal(str)
    dsrtraj_runstat = pyqtSignal(str)
    setrunflag = pyqtSignal(bool)
    setapflag = pyqtSignal(bool)
    setfwf = pyqtSignal(bool)
    setbwf = pyqtSignal(bool)
    setbtuvclamp = pyqtSignal(bool)
    

    def __init__(self, parent = None):
        super(Thread_ConfRoomDisfection, self).__init__(parent)
        
        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)

        rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)

        self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRSetTraj, queue_size = 10)

        rospy.Subscriber("/status_sdr_traj", ZBDSRTrajStatus, self.CallbackRunTrajStatus)

        rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)
        
        rospy.Subscriber("/EmergencyStop", UInt8, self.Callback_EStop) 

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

        self.naviStatus = NavigationControlStatus.IDLING
        self.prevnaviStatus = NavigationControlStatus.IDLING
        self.trajStatus = ZBDSRSetTraj.IDLING

        self.csStatus = "none"
        
        self.CRDIRun = np.chararray(20)
        self.CRDIRun[:] = 0
        
        self.bRunFlag = True
        self.bRunPauseFlag = False

        self.wpName = "none"
        
        self.RepCnt = 5
        
        self.bEStop_Status = 0
        self.bEStop = False
        self.bprevEStop = False
        self.bCSParking = False
        

        
    def Callback_EStop(self, msg):
        self.bEStop_Status = msg.data
        #print("EStop Status : {}\r".format(self.bEStop_Status))
        
        if self.bEStop_Status != 1 and self.bEStop == False:
        # if self.bEStop_Status != 0 and self.bEStop == False:
            if self.bEStop_Status == 0:
                print("Detected Lidar Field\r")
            elif self.bEStop_Status == 3:
                print("Pushed E-Stop Button\r")
            elif self.bEStop_Status == 2:
                print("Detected Lidar Field and Pushed E-Stop Button\r")

            self.bEStop = True

        elif self.bEStop == True and self.bEStop_Status == 1:
        # elif self.bEStop == True and self.bEStop_Status == 0:
            print("Release E-stop/ Lidar detecting\r")
            
            self.bEStop = False
    
    
    def StopWaypoint(self):
        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.wpName
        self.ncpub.publish(nc)
        mstr = "[WP] Stop way point : " + self.wpName
        print(mstr)
        self.navi_runstat.emit(mstr)

    def CallbackCSStatus(self, request):
        if self.bCSParking == True:
            self.csStatus = request.data

            mstr = "Charging Station Status : " + self.csStatus

            self.navi_runstat.emit(mstr)
        

    def CallbackRunNaviCtrlStatus(self, request):
        self.naviStatus = request.status

        if(self.prevnaviStatus != self.naviStatus):
            mstr = request.status_description
            print(mstr)
            
            self.navi_runstat.emit(mstr)

            self.prevnaviStatus = self.naviStatus


    def CallbackRunTrajStatus(self, request):

        stat_str = "none"
        
        if request.curstatus == ZBDSRTrajStatus.IDLING:
            stat_str = "Idling"
            self.trajStatus = ZBDSRTrajStatus.IDLING
        elif request.curstatus == ZBDSRTrajStatus.RUNNING:
            stat_str = "Running"
            self.trajStatus = ZBDSRTrajStatus.RUNNING
        elif request.curstatus == ZBDSRTrajStatus.PAUSED:
            stat_str = "Paused"
            self.trajStatus = ZBDSRTrajStatus.PAUSED
        elif request.curstatus == ZBDSRTrajStatus.COMPLETED:
            stat_str = "Completed"
            self.trajStatus = ZBDSRTrajStatus.COMPLETED
        elif request.curstatus == ZBDSRTrajStatus.CANCELLED:
            stat_str = "Canceled"
            self.trajStatus = ZBDSRTrajStatus.CANCELLED
        elif request.curstatus == ZBDSRTrajStatus.START:
            stat_str = "Start"
            self.trajStatus = ZBDSRTrajStatus.START
        elif request.curstatus == ZBDSRTrajStatus.STOP:
            stat_str = "Stop"
            self.trajStatus = ZBDSRTrajStatus.STOP
        elif request.curstatus == ZBDSRTrajStatus.ESTOP:
            stat_str = "EStop"
            self.trajStatus = ZBDSRTrajStatus.ESTOP
        elif request.curstatus == ZBDSRTrajStatus.RESTART:
            stat_str = "Restart"
            self.trajStatus = ZBDSRTrajStatus.RESTART

        mstr = "TrajNum(" + str(request.trajnum) + "), Status:" + stat_str
        print(mstr)
        
        self.dsrtraj_runstat.emit(mstr)
        

    def gotoWayPoint(self, wpname):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = wpname

        self.wpName = wpname

        self.ncpub.publish(nc)
              
        mstr = "[WP] Start conference room disinfection way point : " + nc.goal_name
        print(mstr)

    def RunDSRTraj(self, trajnum):
        zbdsr_traj = ZBDSRSetTraj()
        zbdsr_traj.trajnum = trajnum
        zbdsr_traj.setstatus = ZBDSRSetTraj.START
        self.dsrpub.publish(zbdsr_traj)
        
    def stop(self):
        self.quit()
        self.wait(5000)
        
    def setRunFlag(self, val):
        self.bRunFlag = val
        
    def setPauseFlag(self, val):
        self.bRunPauseFlag = val

    def PumpOn(self):
        cmdstr = "gnome-terminal -- rostopic pub /pump std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
   
    def PumpOff(self):
        cmdstr = "gnome-terminal -- rostopic pub /pump std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)

    def SolOn(self):
        cmdstr = "gnome-terminal -- rostopic pub /sol std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)

    def SolOff(self):
        cmdstr = "gnome-terminal -- rostopic pub /sol std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)

    def CheckEStop(self):

        if self.bEStop == True:

            while True:

                if self.bEStop == False:
                    break

                sleep(0.2)

    def CheckAborted(self, wp):
        if self.naviStatus == NavigationControlStatus.ABORTED:
            self.navi_runstat.emit("Navi. Aborted")

            twist = Twist()
            twist.linear.x = -0.04; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(2)

            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(2)

            self.StopWaypoint()

            sleep(0.5)

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
            sleep(1)

            self.gotoWayPoint(wp)
        
    def run(self):
        runcnt = 0
        
        bErrFlag = False

        self.setlaser.emit(True)

        if self.bRunFlag == True:

            self.naviStatus = NavigationControlStatus.IDLING
            
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
            
            # ========================================
            # Conference Room Disinfection Process #1
            # ========================================

            self.gotoWayPoint("CRWPDoor")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("CRWPDoor")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRWPDoor CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRWPDoor CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRWPDoor CSDI process...")
                            break
                    
                        sleep(0.1)

                    
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[0] = 1
                    
                    print("Arrived at CRWPDoor...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("CRWPDoor")

                        self.navi_runstat.emit("Restart CRWPDoor")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[0] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRWPDoor")

                        break

                sleep(0.1)

        
        # ========================================
        # Conference Room Disinfection Process #2
        # ========================================
        if self.bRunFlag == True:

            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
            
            self.gotoWayPoint("CRWP1")

            sleep(5)


            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("CRWP1")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRWP1 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRWP1 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRWP1 CSDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[1] = 1
                    
                    print("Arrived at CRWP1...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("CRWP1")

                        self.navi_runstat.emit("Restart CRWP1")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[1] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRWP1")

                        break

                sleep(0.1)

            
            if self.bRunFlag == True:
            #if bErrFlag == False and self.bRunFlag == True:
                self.trajStatus = ZBDSRTrajStatus.IDLING
                sleep(2)

                self.setbtuvclamp.emit(True)
                sleep(1.0)

                # self.setfwf.emit(True)
                # print("Turn on Front WP")

                # self.CheckEStop()

                # sleep(0.5)

                # self.setbwf.emit(True)
                # print("Turn on Rear WP")

                # sleep(3)

                # self.CheckEStop()

                self.RunDSRTraj(5)

                sleep(10)

                print("Moving Trjaectory #5...")

                # while True:
                #     if self.bEStop == True:
                #         continue

                #     if self.bEStop == False and self.bprevEStop == True:
                #         print("====> delay 5 sec...")
                #         sleep(5)
                #         #sleep(10)
                #         self.bprevEStop = False
                    
                #     if self.bRunFlag == False:
                #         self.stopTraj()

                #         self.dsrtraj_runstat.emit("Stop moving HDI traj. #9...")
                            
                #         sleep(1)
                            
                #         self.stop()

                #         break
                            
                #     if self.trajStatus == ZBDSRTrajStatus.COMPLETED: 
                #         break

                #     sleep(0.1)

                # if self.bRunFlag == True:
                #     print("Complete moving HDI Trjaectory #10...")        
                    
                #     self.CheckEStop()

                #     self.setfwf.emit(False)
                #     print("Turn off Front WP")

                #     sleep(0.5)

                #     self.CheckEStop()

                #     self.setbwf.emit(False)
                #     print("Turn off Rear WP")

                #     sleep(2)
        
              
        # ========================================
        # Conference Room Disinfection Process #3
        # ========================================
        
        if self.bRunFlag == True:
        #if bErrFlag == False and self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
            
            self.gotoWayPoint("DSOLON1")

            sleep(5)
 
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("DSOLON1")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop DSOLON1 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause DSOLON1 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart DSOLON1 CSDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[1] = 1
                    
                    print("Arrived at DSOLON1...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("DSOLON1")

                        self.navi_runstat.emit("Restart DSOLON1")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[1] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) DSOLON1")

                        break

                sleep(0.1)        

            if self.bRunFlag == True:
                self.PumpOn()

                print("Turn on Pump...")
                
                sleep(1)

                self.SolOn()
                
                print("Turn on Sol. valve...")
       
        # ========================================
        # Conference Room Disinfection Process #4
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("DSOLOFF1")

            sleep(5)
 
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("DSOLOFF1")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop DSOLOFF1 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause DSOLOFF1 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart DSOLOFF1 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at DSOLOFF1...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("DSOLOFF1")

                        self.navi_runstat.emit("Restart DSOLOFF1")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) DSOLOFF1")

                        break

                sleep(0.1)

            if self.bRunFlag == True:      
                self.SolOff()
                
                print("Turn off Sol. valve...")
                
                self.PumpOff()

                print("Turn off Pump...")


        # ========================================
        # Conference Room Disinfection Process #5
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("CRWP2")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue
                
                self.CheckAborted("CRWP2")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRWP2 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRWP2 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRWP2 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at CRWP2...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("CRWP2")

                        self.navi_runstat.emit("Restart CRWP2")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRWP2")

                        break

                sleep(0.1)

        # ========================================
        # Conference Room Disinfection Process #5
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("CRWP2_1")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue
                
                self.CheckAborted("CRWP2_1")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRWP2_1 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRWP2_1 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRWP2_1 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at CRWP2_1...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("CRWP2_1")

                        self.navi_runstat.emit("Restart CRWP2_1")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRWP2_1")

                        break

                sleep(0.1)
               
        # ========================================
        # Conference Room Disinfection Process #6
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("DSOLON2")

            sleep(5)
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("DSOLON2")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop DSOLON2 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause DSOLON2 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart DSOLON2 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at DSOLON2...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("DSOLON2")

                        self.navi_runstat.emit("Restart DSOLON2")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) DSOLOFF1")

                        break

                sleep(0.1)

            if self.bRunFlag == True:
                self.PumpOn()

                print("Turn on Pump...")
                
                sleep(1)

                self.SolOn()
                
                print("Turn on Sol. valve...")
       

        # ========================================
        # Conference Room Disinfection Process #7
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("DSOLOFF2")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("DSOLOFF2")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop DSOLOFF2 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause DSOLOFF2 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart DSOLOFF2 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at DSOLOFF2...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("DSOLOFF2")

                        self.navi_runstat.emit("Restart DSOLOFF2")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) DSOLOFF2")

                        break

                sleep(0.1)

            if self.bRunFlag == True:
                self.SolOff()
                
                print("Turn off Sol. valve...")
                
                self.PumpOff()

                print("Turn off Pump...")
        # ========================================
        # Conference Room Disinfection Process #8
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("CRWP3")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue
                
                self.CheckAborted("CRWP3")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRWP3 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRWP3 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRWP3 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at CRWP3...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("CRWP3")

                        self.navi_runstat.emit("Restart CRWP3")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRWP3")

                        break

                sleep(0.1)

        # ========================================
        # Conference Room Disinfection Process #9
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("CRWP4")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue
                
                self.CheckAborted("CRWP4")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRWP4 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRWP4 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRWP4 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at CRWP4...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("CRWP4")

                        self.navi_runstat.emit("Restart CRWP4")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRWP4")

                        break

                sleep(0.1)

        # ========================================
        # Conference Room Disinfection Process #10
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("DSOLON3")

            sleep(5)
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("DSOLON3")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop DSOLON3 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause DSOLON3 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart DSOLON3 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at DSOLON3...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("DSOLON3")

                        self.navi_runstat.emit("Restart DSOLON3")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) DSOLOFF3")

                        break

                sleep(0.1)

            if self.bRunFlag == True:
                self.PumpOn()

                print("Turn on Pump...")
                
                sleep(1)

                self.SolOn()
                
                print("Turn on Sol. valve...")
       

        # ========================================
        # Conference Room Disinfection Process #11
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("DSOLOFF3")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("DSOLOFF3")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop DSOLOFF3 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause DSOLOFF3 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart DSOLOFF3 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at DSOLOFF3...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("DSOLOFF3")

                        self.navi_runstat.emit("Restart DSOLOFF3")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) DSOLOFF3")

                        break

                sleep(0.1)

            if self.bRunFlag == True:
                self.SolOff()
                
                print("Turn off Sol. valve...")
                
                self.PumpOff()

                print("Turn off Pump...")

        # ========================================
        # Conference Room Disinfection Process #12
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("CRWP5")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue
                
                self.CheckAborted("CRWP5")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRWP5 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRWP5 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRWP5 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at CRWP5...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("CRWP5")

                        self.navi_runstat.emit("Restart CRWP5")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRWP5")

                        break

                sleep(0.1)

        # ========================================
        # Conference Room Disinfection Process #13
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("DSOLON4")

            sleep(5)
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("DSOLON4")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop DSOLON4 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause DSOLON4 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart DSOLON4 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at DSOLON4...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("DSOLON4")

                        self.navi_runstat.emit("Restart DSOLON4")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) DSOLOFF4")

                        break

                sleep(0.1)

            if self.bRunFlag == True:
                self.PumpOn()

                print("Turn on Pump...")
                
                sleep(1)

                self.SolOn()
                
                print("Turn on Sol. valve...")
       

        # ========================================
        # Conference Room Disinfection Process #14
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("DSOLOFF4")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("DSOLOFF4")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop DSOLOFF4 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause DSOLOFF4 CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart DSOLOFF4 CSDI process...")
                            break
                    
                        sleep(0.1)
                
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[2] = 1              
                    
                    print("Arrived at DSOLOFF4...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("DSOLOFF4")

                        self.navi_runstat.emit("Restart DSOLOFF4")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) DSOLOFF4")

                        break

                sleep(0.1)

            if self.bRunFlag == True:
                self.SolOff()
                
                print("Turn off Sol. valve...")
                
                self.PumpOff()

                print("Turn off Pump...")

        # ========================================
        # Conference Room Disinfection Process #15
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("CRWP6")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("CRWP6")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRWP1 CRWP6 process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRWP1 CRWP6 process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRWP6 CSDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[3] = 1
                    
                    print("Arrived at CRWP6...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("CRWP6")

                        self.navi_runstat.emit("Restart CRWP6")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[3] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRWP6")

                        break

                sleep(0.1)

                
            if self.bRunFlag == True:
            #if bErrFlag == False and self.bRunFlag == True:
            
                # self.onSolOffPBtn()
                
                print("Turn on Sol. valve...")
                
                # self.pumpOffPB()

                print("Turn on Pump...")

                self.setbtuvclamp.emit(False)
                sleep(1.0)

                zbdsr_traj = ZBDSRSetTraj()            
                zbdsr_traj.trajnum = 5
                zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                self.dsrpub.publish(zbdsr_traj)

                sleep(2)

                # self.setfwf.emit(True)
                # print("Turn on Front WP")

                # self.CheckEStop()

                # sleep(0.5)

                # self.setbwf.emit(True)
                # print("Turn on Rear WP")

                # sleep(3)

                # self.CheckEStop()

                self.trajStatus = ZBDSRTrajStatus.IDLING
                #sleep(2)

                self.RunDSRTraj(6)

                print("[CRWP3] Send topic [Run Trajectory 6]")

                while True:
                    if self.bEStop == True:
                        continue

                    if self.bEStop == False and self.bprevEStop == True:
                        print("====> delay 5 sec...")
                        sleep(5)
                        self.bprevEStop = False
                    
                    if self.bRunFlag == False:
                        self.stopTraj()

                        self.dsrtraj_runstat.emit("Stop moving HDI traj. #9...")
                            
                        sleep(1)
                            
                        self.stop()

                        break

                    if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                        
                        break

                    sleep(0.1)


                print("Complete moving Trjaectory #6...")

                # if self.bRunFlag == True:
                #     print("Complete moving HDI Trjaectory #10...")        
                    
                #     self.CheckEStop()

                #     self.setfwf.emit(False)
                #     print("Turn off Front WP")

                #     sleep(0.5)

                #     self.CheckEStop()

                #     self.setbwf.emit(False)
                #     print("Turn off Rear WP")

                #     sleep(2)
        
        
        # # ========================================
        # # Conference Room Disinfection Process #9
        # # ========================================
        # if self.bRunFlag == True:
        #     self.trajStatus = ZBDSRTrajStatus.IDLING

        #     sleep(2)

        #     self.RunDSRTraj(7)

        #     sleep(8)

        #     print("Complete moving DSR Trajaectory #7...")
            
        #     sleep(2)
            
        #     self.PumpOn()

        #     print("Turn on Pump...")
            
        #     sleep(1)

        #     self.SolOn()
            
        #     print("Turn on Sol. valve...")
            
        #     # ==========================================
        #     # Conference Room Disinfection Process #10
        #     # ==========================================
            
        #     self.naviStatus = NavigationControlStatus.IDLING
            
        #     os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #     self.gotoWayPoint("CRTRAJ1")

        #     sleep(5)

        #     runcount = 0

        #     while True:
        #         if self.bEStop == True:
        #             continue
                    
        #         if self.bRunFlag == False:
        #             self.StopWaypoint()
                    
        #             self.navi_runstat.emit("Stop CRTRAJ1 CSDI process...")

        #             self.setrunflag.emit(True)

        #             sleep(1)
                    
        #             self.stop()

        #             break
            
        #         if self.naviStatus == NavigationControlStatus.TRAJCOMPLETED:

        #             print("navi Status : " + str(self.naviStatus))

        #             self.naviStatus = NavigationControlStatus.IDLING

        #             break

        #         sleep(0.1)

        #     if self.bRunFlag == True:
        #         print("======>>>>> Complete driving trajectory #1...")
                

        #         self.SolOff()
                
        #         print("Turn on Sol. valve...")
                
        #         self.PumpOff()

        #         print("Turn on Pump...")


        #         zbdsr_traj = ZBDSRSetTraj()            
        #         zbdsr_traj.trajnum = 7
        #         zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
        #         self.dsrpub.publish(zbdsr_traj)

        #         sleep(2)

        #         self.trajStatus = ZBDSRTrajStatus.IDLING
        #         sleep(2)

        #         self.RunDSRTraj(8)

        #         while True:
        #             if self.bEStop == True:
        #                 continue

        #             if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                        
        #                 break

        #             sleep(0.1)

        #         print("Complete moving Trjaectory #8...")
        
        
        # ========================================
        # Conference Room Disinfection Process #16
        # ========================================

        if self.bRunFlag == True:
            self.naviStatus = NavigationControlStatus.IDLING

            self.gotoWayPoint("CRExitPos")

            sleep(5)
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("CRExitPos")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRExitPos CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRExitPos CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRExitPos CSDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[4] = 1
                    
                    print("Arrived at CRExitPos...")
                    break
                    
                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("CRExitPos")

                        self.navi_runstat.emit("Restart CRExitPos")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.CRDIRun[4] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRExitPos")

                        break

                sleep(0.1)
            
            
        # ========================================
        # Conference Room Disinfection Process #17
        # ========================================
        
        if self.bRunFlag == True:
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("chargingpos")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("chargingpos")
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop chargingpos CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause chargingpos CSDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart chargingpos CSDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    
                    self.CRDIRun[5] = 1

                    print("Arrived at chargingpos...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("chargingpos")

                        self.navi_runstat.emit("Restart chargingpos")

                    else:
                        self.CRDIRun[5] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) chargingpos")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) chargingpos")

                    self.HDIRun[5] = 0

                sleep(0.1)

            if self.bRunFlag == True:
                print("Arrived at charging position...")           

        # ========================================
        # Conference Room Disinfection Process #18
        # ========================================

        if self.bRunFlag == True:

            client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            client.wait_for_server()

            self.setlaser.emit(False)

            goal = ChargingGoal()
            client.send_goal(goal)

            print("Parking charging station...")

            while True:
                if self.bEStop == True:
                    continue
                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop auto parking CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break

                if self.csStatus == "contact":
                    print("====>csStatus : " + self.csStatus)
                    self.navi_runstat.emit("Complete parking charging station.")

                    self.setapflag.emit(True)
                    self.setrunflag.emit(False)                   

                    break
                    
                sleep(0.1)
        
            if self.bRunFlag == True:
                print("Complete all disinfection processes in the conference room.")
    
        else:
            print("Stop CSDI processing!!!")

        self.stop()




class Thread_HollowDisfection(QThread):
    navi_runstat = pyqtSignal(str)
    dsrtraj_runstat = pyqtSignal(str)
    setrunflag = pyqtSignal(bool)
    setapflag = pyqtSignal(bool)
    setfwf = pyqtSignal(bool)
    setbwf = pyqtSignal(bool)
    setuvlamp =  pyqtSignal(bool)
    setlaser = pyqtSignal(bool)
    setpcsbtn = pyqtSignal(bool)        # 2022. 03. 03
    
    # getcurtraj = pyqtSignal(int)

    def __init__(self, parent = None):
        super(Thread_HollowDisfection, self).__init__(parent)
        
        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)

        rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)

        self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRSetTraj, queue_size = 10)

        rospy.Subscriber("/status_sdr_traj", ZBDSRTrajStatus, self.CallbackRunTrajStatus)

        rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)
        
        rospy.Subscriber("/EmergencyStop", UInt8, self.Callback_EStop) 

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

        self.naviStatus = NavigationControlStatus.IDLING
        self.prevnaviStatus = NavigationControlStatus.IDLING
        self.trajStatus = ZBDSRSetTraj.IDLING

        self.HDIRun = np.chararray(15)
        self.HDIRun[:] = 0
        
        self.bRunFlag = True
        self.bRunPauseFlag = False

        self.csStatus = "none"

        self.wpName = "none"
        
        self.RepCnt = 5
        
        self.bEStop_Status = 0
        self.bEStop = False
        self.bprevEStop = False
        self.bCSParking = False

        self.cur_trajnum = 0

        
    def Callback_EStop(self, msg):
        self.bEStop_Status = msg.data
        #print("EStop Status : {}\r".format(self.bEStop_Status))
        
        if self.bEStop_Status != 1 and self.bEStop == False:
        # if self.bEStop_Status != 0 and self.bEStop == False:
            if self.bEStop_Status == 0:
                print("Detected Lidar Field\r")
            elif self.bEStop_Status == 3:
                print("Pushed E-Stop Button\r")
            elif self.bEStop_Status == 2:
                print("Detected Lidar Field and Pushed E-Stop Button\r")

            self.bprevEStop = self.bEStop
            self.bEStop = True

        elif self.bEStop == True and self.bEStop_Status == 1:
        # elif self.bEStop == True and self.bEStop_Status == 0:
            print("Release E-stop/ Lidar detecting\r")
            
            self.bprevEStop = self.bEStop
            self.bEStop = False
            
            
    def CallbackRunNaviCtrlStatus(self, request):
        self.naviStatus = request.status

        if(self.prevnaviStatus != self.naviStatus):
            mstr = request.status_description
            print(mstr)

            print("naviStatus : "+str(self.naviStatus))

            self.navi_runstat.emit(mstr)

            self.prevnaviStatus = self.naviStatus

    def CallbackCSStatus(self, request):
        if self.bCSParking == True:
            self.csStatus = request.data

            mstr = "Charging Station Status : " + self.csStatus

            self.navi_runstat.emit(mstr)


    def CallbackRunTrajStatus(self, request):

        stat_str = "none"
        
        if request.curstatus == ZBDSRTrajStatus.IDLING:
            stat_str = "Idling"
            self.trajStatus = ZBDSRTrajStatus.IDLING
        elif request.curstatus == ZBDSRTrajStatus.RUNNING:
            stat_str = "Running"
            self.trajStatus = ZBDSRTrajStatus.RUNNING
        elif request.curstatus == ZBDSRTrajStatus.PAUSED:
            stat_str = "Paused"
            self.trajStatus = ZBDSRTrajStatus.PAUSED
        elif request.curstatus == ZBDSRTrajStatus.COMPLETED:
            stat_str = "Completed"
            self.trajStatus = ZBDSRTrajStatus.COMPLETED
        elif request.curstatus == ZBDSRTrajStatus.CANCELLED:
            stat_str = "Canceled"
            self.trajStatus = ZBDSRTrajStatus.CANCELLED
        elif request.curstatus == ZBDSRTrajStatus.START:
            stat_str = "Start"
            self.trajStatus = ZBDSRTrajStatus.START
        elif request.curstatus == ZBDSRTrajStatus.STOP:
            stat_str = "Stop"
            self.trajStatus = ZBDSRTrajStatus.STOP
        elif request.curstatus == ZBDSRTrajStatus.ESTOP:
            stat_str = "EStop"
            self.trajStatus = ZBDSRTrajStatus.ESTOP
        elif request.curstatus == ZBDSRTrajStatus.RESTART:
            stat_str = "Restart"
            self.trajStatus = ZBDSRTrajStatus.RESTART
            

        mstr = "TrajNum(" + str(request.trajnum) + "), Status:" + stat_str
        print(mstr)
        
        self.dsrtraj_runstat.emit(mstr)

    def StopWaypoint(self):
        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.wpName
        self.ncpub.publish(nc)
        mstr = "[WP] Stop way point : " + self.wpName
        print(mstr)        
        self.navi_runstat.emit(mstr)

        
    def gotoWayPoint(self, wpname):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = wpname
        self.ncpub.publish(nc)
        
        self.wpName = wpname
        
        mstr = "[WP] Start hollow disinfection way point : " + nc.goal_name
        print(mstr)

    def RunDSRTraj(self, trajnum):
        zbdsr_traj = ZBDSRSetTraj()
        self.cur_trajnum = trajnum
        zbdsr_traj.trajnum = trajnum
        zbdsr_traj.setstatus = ZBDSRSetTraj.START
        self.dsrpub.publish(zbdsr_traj)
        
    def setRunFlag(self, val):
        self.bRunFlag = val
        print("RunFlag : " + str(val))
        
    def setPauseFlag(self, val):
        self.bRunPauseFlag = val
        print("PauseFlag : " + str(val))
        
    def stopTraj(self):
        zbdsr_traj = ZBDSRSetTraj()            
        zbdsr_traj.trajnum = self.cur_trajnum
        zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
        self.dsrpub.publish(zbdsr_traj)

        mstr = "Stop DSR Robot Trajectory Num : " + str(zbdsr_traj.trajnum)
        self.navi_runstat.emit(mstr)

    # def BackwardMove(self):
    #     twist = Twist()
    #     twist.linear.x = -0.04; twist.linear.y = 0; twist.linear.z = 0
    #     twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
    #     self.pub_twist.publish(twist)

    #     sleep(10)

    #     twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
    #     twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
    #     self.pub_twist.publish(twist)


    # def UVLampOn(self):
    #     cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: true\" -1 "
    #     #print(cmdstr)
    #     os.system(cmdstr)

    #     print("Turn on UV Lamp...")

    # def UVLampOff(self):
    #     cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: false\" -1 "
    #     #print(cmdstr)
    #     os.system(cmdstr)

    #     print("Turn off UV Lamp...")

    def CheckEStop(self):

        if self.bEStop == True:

            while True:

                if self.bEStop == False:
                    break

                sleep(0.2)

    def CheckAborted(self, wp):
        if self.naviStatus == NavigationControlStatus.ABORTED:
            self.navi_runstat.emit("Navi. Aborted")

            twist = Twist()
            twist.linear.x = -0.04; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(2)

            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(2)

            self.StopWaypoint()

            sleep(0.5)

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            sleep(1)

            self.gotoWayPoint(wp)

                        
    def navi_Process(self):

        runcnt = 0

        bErrFlag = False

        self.setlaser.emit(True)

        # //////////////////////////////////////////
        #               DEVICE 1
        # //////////////////////////////////////////

        # =================================
        # Hallow Disinfection Process #1
        # =================================
        
        if (self.HDIRun[0] == '0') and self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device1")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    # print("EStop ==>")
                    continue

                self.CheckAborted("device1")

                # if self.naviStatus == NavigationControlStatus.ABORTED:
                #     self.navi_runstat.emit("Navi. Aborted")

                #     self.BackwardMove()

                #     sleep(2)

                #     self.gotoWayPoint("device1")


                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop device1 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:

                    self.navi_runstat.emit("Pause device1 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart device1 HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    #self.HDIRun[2] = 1

                    print("Arrived at device1...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device1")

                        self.navi_runstat.emit("Restart device1")

                        print("Restart Device1 : " + str(runcnt))

                        sleep(2)

                    else:
                        #self.HDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) device1")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) device1")

                    #self.HDIRun[2] = 0

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(1)

        # =================================
        # Hallow Disinfection Process #1_1
        # =================================

        if self.HDIRun[0] == '0' and self.bRunFlag == True:     
            self.naviStatus = NavigationControlStatus.IDLING
  
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device1_1")

            sleep(5)

            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device1_1")

                # if self.bEStop == False and self.bprevEStop == True:
                #     sleep(10)
                #     self.bprevEStop = False

                #     self.gotoWayPoint("device1")

                #     self.navi_runstat.emit("Restart Device1")

                #     print("Restart Device1")

                #     self.naviStatus = NavigationControlStatus.IDLING

                    
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop device1_1 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    bErrFlag = True

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause device1_1 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart device1_1 HDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.HDIRun[0] = 1

                    print("Arrived at device1_1...")

                    bErrFlag = False
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device1_1")

                        self.navi_runstat.emit("Restart device1_1")

                        print("Restart device1_1 : " + str(runcnt))

                        self.naviStatus = NavigationControlStatus.IDLING

                        sleep(2)

                    else:
                        self.HDIRun[0] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) device1_1")

                        print("Error(WARNNRGTO) device1_1")

                        bErrFlag = True

                        break

                sleep(0.1)

            if bErrFlag == False and self.bRunFlag == True: 

                #sleep(2)

                self.setfwf.emit(True)
                print("Turn on Front WP")

                sleep(0.5)

                self.CheckEStop()

                self.setbwf.emit(True)
                print("Turn on Rear WP")

                sleep(2.0)
                #sleep(3)

                self.CheckEStop()

                self.trajStatus = ZBDSRTrajStatus.IDLING
                
                self.RunDSRTraj(9)

                print("[device1_1] Send topic [Run Trajectory 9]")

                while True:
                    if self.bEStop == True:
                        continue

                    if self.bEStop == False and self.bprevEStop == True:
                        print("====> delay 5 sec...")
                        sleep(5)
                        #sleep(10)
                        if self.bEStop == False:
                            self.bprevEStop = False
                    
                    if self.bRunFlag == False:
                        self.stopTraj()

                        self.dsrtraj_runstat.emit("Stop moving HDI traj. #9...")
                            
                        sleep(1)
                            
                        self.stop()

                        break
                            
                    # if self.bRunPauseFlag == True:
                    #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #1...")
                    #     while self.bRunPauseFlag:
                    #         if self.bRunPauseFlag is False:
                    #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #1...")
                    #             break
                            
                    #         sleep(0.1)

                    if self.trajStatus == ZBDSRTrajStatus.COMPLETED: 
                        break

                    sleep(0.1)

                if self.bRunFlag == True:

                    self.dsrtraj_runstat.emit("Complete moving HDI Trjaectory #9...")        

                    self.CheckEStop()            

                    self.setuvlamp.emit(True)

                    print("Waiting during disinfection...")

                    self.CheckEStop()

                    sleep(DTRunTime)

                    self.CheckEStop()

                    print("Done first device disinfection...")

                    self.setuvlamp.emit(False)

                    sleep(2)

                    self.CheckEStop()

                    zbdsr_traj = ZBDSRSetTraj()            
                    zbdsr_traj.trajnum = 9
                    zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                    self.dsrpub.publish(zbdsr_traj)

                    sleep(2)

                    self.CheckEStop()

                    self.RunDSRTraj(10)

                    print("[device1_1] Send topic [Run Trajectory 10]")

                    self.trajStatus = ZBDSRTrajStatus.IDLING

                    while True:
                        if self.bEStop == True:
                            continue

                        if self.bEStop == False and self.bprevEStop == True:
                            print("====> delay 5 sec...")
                            sleep(5)
                            self.bprevEStop = False
                            
                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.dsrtraj_runstat.emit("Stop moving HDI traj. #10...")
                            
                            sleep(1)
                            
                            self.stop()

                            break
                            
                        # if self.bRunPauseFlag == True:
                        #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #2...")
                        #     while self.bRunPauseFlag:
                        #         if self.bRunPauseFlag is False:
                        #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #2...")
                        #             break
                            
                        #         sleep(0.1)
                    
                        if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                            break

                        sleep(0.1)

                    if self.bRunFlag == True:
                        print("Complete moving HDI Trjaectory #10...")

                        self.CheckEStop()

                        self.setfwf.emit(False)
                        print("Turn off Front WP")

                        sleep(0.5)

                        self.CheckEStop()

                        self.setbwf.emit(False)
                        print("Turn off Rear WP")

                        sleep(5)
                        #sleep(8)

                        print("Complete first device disinfection...")

            else:
                print("Skip first device disinfection...")

        bErrFlag = False

        # ======================================
        # Hallow Disinfection Process return #1
        # ======================================
        
        if self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device1")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device1")

                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop ret device1 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause ret device1 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart ret device1 HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    # self.HDIRun[2] = 1

                    print("Arrived at ret device1...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device1")

                        self.navi_runstat.emit("Restart ret device1")

                        print("Restart ret Device1 : " + str(runcnt))

                        sleep(2)

                    else:
                        #self.HDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) ret device1")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) ret device1")

                    #self.HDIRun[2] = 0

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(2)

        # //////////////////////////////////////////
        #               DEVICE 2
        # //////////////////////////////////////////

        # =================================
        # Hallow Disinfection Process #2
        # =================================
        
        if (self.HDIRun[1] == '0') and self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device2")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device2")

                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop device2 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause device2 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart device2 HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    #self.HDIRun[2] = 1

                    print("Arrived at device2...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device2")

                        self.navi_runstat.emit("Restart device2")

                        print("Restart Device2 : " + str(runcnt))

                        sleep(2)

                    else:
                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) device2")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) device2")

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(2)

        # =================================
        # Hallow Disinfection Process #2_1
        # =================================

        if self.HDIRun[1] == '0' and self.bRunFlag == True:   
            #self.trajStatus = ZBDSRTrajStatus.IDLING
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device2_1")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device2_1")

                if self.bRunFlag == False:
                    self.StopWaypoint()

                    self.navi_runstat.emit("Stop device2_1 HDI process...")

                    self.setrunflag.emit(True)
                    
                    sleep(1)
                    
                    self.stop()

                    bErrFlag = True

                    break                    
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause device2_1 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart device2_1 HDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.HDIRun[1] = 1

                    print("Arrived at device2_1...")

                    bErrFlag = False
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device2_1")

                        self.navi_runstat.emit("Restart device2_1")

                        print("Restart device2_1 : " + str(runcnt))

                        self.naviStatus = NavigationControlStatus.IDLING

                        sleep(2)

                    else:
                        self.HDIRun[1] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) device2_1")

                        bErrFlag = True

                        break

                sleep(0.1)


            if bErrFlag == False and self.bRunFlag == True:
                self.trajStatus = ZBDSRTrajStatus.IDLING

                self.setfwf.emit(True)
                print("Turn on Front WP")

                sleep(0.5)

                self.CheckEStop()

                self.setbwf.emit(True)
                print("Turn on Rear WP")

                sleep(2.0)
                # sleep(2)

                self.CheckEStop()

                self.RunDSRTraj(9)

                print("[device2_1] Send topic [Run Trajectory 9]")

                while True:
                    if self.bEStop == True:
                        continue

                    if self.bEStop == False and self.bprevEStop == True:
                        print("====> delay 5 sec...")
                        sleep(5)
                        self.bprevEStop = False

                    if self.bRunFlag == False:
                        self.stopTraj()

                        self.dsrtraj_runstat.emit("Stop moving HDI traj. #9...")
                            
                        sleep(1)
                            
                        self.stop()

                        break
                            
                    # if self.bRunPauseFlag == True:
                    #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #1...")
                    #     while self.bRunPauseFlag:
                    #         if self.bRunPauseFlag is False:
                    #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #1...")
                    #             break
                            
                    #         sleep(0.1)

                    if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                        break

                    sleep(0.1)

                if self.bRunFlag == True:
                    print("Complete moving HDI Trjaectory #9...")

                    self.CheckEStop()

                    self.setuvlamp.emit(True)

                    print("Waiting during disinfection...")

                    self.CheckEStop()

                    sleep(DTRunTime)

                    print("Done first device disinfection...")

                    self.CheckEStop()

                    self.setuvlamp.emit(False)

                    sleep(2)

                    self.CheckEStop()

                    zbdsr_traj = ZBDSRSetTraj()            
                    zbdsr_traj.trajnum = 9
                    zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                    self.dsrpub.publish(zbdsr_traj)

                    sleep(2)

                    self.CheckEStop()

                    self.RunDSRTraj(10)

                    print("[device2_1] Send topic [Run Trajectory 10]")

                    self.trajStatus = ZBDSRTrajStatus.IDLING

                    while True:
                        if self.bEStop == True:
                            continue

                        if self.bEStop == False and self.bprevEStop == True:
                            print("====> delay 5 sec...")
                            sleep(5)
                            self.bprevEStop = False

                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.dsrtraj_runstat.emit("Stop moving HDI traj. #10...")
                                
                            sleep(1)
                                
                            self.stop()

                            break
                                
                        # if self.bRunPauseFlag == True:
                        #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #2...")
                        #     while self.bRunPauseFlag:
                        #         if self.bRunPauseFlag is False:
                        #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #2...")
                        #             break
                                
                        #         sleep(0.1)
                    
                        if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                            break

                        sleep(0.1)

                    if self.bRunFlag == True:
                        print("Complete moving HDI Trjaectory #10...")        
                        
                        self.CheckEStop()

                        self.setfwf.emit(False)
                        print("Turn off Front WP")

                        sleep(0.5)

                        self.CheckEStop()

                        self.setbwf.emit(False)
                        print("Turn off Rear WP")

                        sleep(5)

                        print("Complete second device disinfection...")

            else:
                print("Skip second device disinfection...")

        bErrFlag = False
        
        # ======================================
        # Hallow Disinfection Process return #2
        # ======================================
        if self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device2")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device2")

                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop ret device2 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause ret device2 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart ret device2 HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    #self.HDIRun[2] = 1

                    print("Arrived at ret device2...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device2")

                        self.navi_runstat.emit("Restart ret device2")

                        print("Restart ret Device2 : " + str(runcnt))

                        sleep(2)

                    else:
                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) ret device2")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) device2")

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(2)
        

        # //////////////////////////////////////////
        #               DEVICE 3
        # //////////////////////////////////////////

        # =================================
        # Hallow Disinfection Process #3
        # =================================
        
        # if (self.HDIRun[2] == '0') and self.bRunFlag == True:   
        #     self.naviStatus = NavigationControlStatus.IDLING

        #     os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #     self.gotoWayPoint("device3")

        #     sleep(5)

        #     runcnt = 0
            
        #     while True:
        #         if self.bEStop == True:
        #             continue

        #         if self.bRunFlag == False:
        #             self.stopTraj()

        #             self.navi_runstat.emit("Stop device3 HDI process...")

        #             self.setrunflag.emit(True)

        #             sleep(1)

        #             self.stop()

        #             bErrFlag = True

        #             break

        #         if self.bRunPauseFlag == True:
        #             self.navi_runstat.emit("Pause device3 HDI process...")
        #             while self.bRunPauseFlag:
        #                 if self.bRunPauseFlag is False:
        #                     self.navi_runstat.emit("Restart device3 HDI process...")
        #                     break
                    
        #                 sleep(0.1)            
        
        #         if self.naviStatus == NavigationControlStatus.COMPLETED:
        #             #self.HDIRun[2] = 1

        #             print("Arrived at device3...")
        #             break

        #         if self.naviStatus == NavigationControlStatus.WARNNRGTO:
        #             runcnt = runcnt + 1

        #             if runcnt <= self.RepCnt:
        #                 os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #                 self.gotoWayPoint("device3")

        #                 self.navi_runstat.emit("Restart device3")

        #                 print("Restart Device3 : " + str(runcnt))

        #                 sleep(2)

        #             else:
        #                 runcnt = 0

        #                 self.navi_runstat.emit("Error(WARNNRGTO) device3")

        #             break

        #         if self.naviStatus == NavigationControlStatus.ERRGTGF:
        #             self.navi_runstat.emit("Error(ERRGTGF) device3")

        #         sleep(0.1)


        #     if self.bRunFlag == True:
        #         sleep(2)


        # =================================
        # Hallow Disinfection Process #3_1
        # =================================

        # if self.HDIRun[2] == '0' and self.bRunFlag == True:   
        #     #self.trajStatus = ZBDSRTrajStatus.IDLING
        #     self.naviStatus = NavigationControlStatus.IDLING

        #     os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #     self.gotoWayPoint("device3_1")

        #     sleep(5)

        #     runcnt = 0
            
        #     while True:
        #         if self.bEStop == True:
        #             continue

        #         if self.bRunFlag == False:
        #             self.StopWaypoint()

        #             self.navi_runstat.emit("Stop device3_1 HDI process...")

        #             self.setrunflag.emit(True)
                    
        #             sleep(1)
                    
        #             self.stop()

        #             bErrFlag = True

        #             break                    
                    
        #         if self.bRunPauseFlag == True:
        #             self.navi_runstat.emit("Pause device3_1 HDI process...")
        #             while self.bRunPauseFlag:
        #                 if self.bRunPauseFlag is False:
        #                     self.navi_runstat.emit("Restart device3_1 HDI process...")
        #                     break
                    
        #                 sleep(0.1)
            
        #         if self.naviStatus == NavigationControlStatus.COMPLETED:
        #             self.HDIRun[2] = 1

        #             print("Arrived at device3_1...")

        #             bErrFlag = False
        #             break

        #         if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
        #             runcnt = runcnt + 1

        #             if runcnt <= self.RepCnt:
        #                 os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #                 self.gotoWayPoint("device3_1")

        #                 self.navi_runstat.emit("Restart device3_1")

        #                 print("Restart device3_1 : " + str(runcnt))

        #                 self.naviStatus = NavigationControlStatus.IDLING

        #                 sleep(2)

        #             else:
        #                 self.HDIRun[2] = 0

        #                 runcnt = 0

        #                 self.navi_runstat.emit("Error(WARNNRGTO) device3_1")

        #                 bErrFlag = True

        #                 break

        #         sleep(0.1)


        #     if bErrFlag == False and self.bRunFlag == True:
        #         self.trajStatus = ZBDSRTrajStatus.IDLING

        #         self.setfwf.emit(True)
        #         print("Turn on Front WP")

        #         self.setbwf.emit(True)
        #         print("Turn on Rear WP")

        #         sleep(5)
        #         # sleep(2)

        #         self.RunDSRTraj(9)

        #         print("[device3_1] Send topic [Run Trajectory 9]")

        #         while True:
        #             if self.bEStop == True:
        #                 continue

        #             if self.bEStop == False and self.bprevEStop == True:
        #                 print("====> delay 10 sec...")
        #                 sleep(10)
        #                 self.bprevEStop = False

        #             if self.bRunFlag == False:
        #                 self.stopTraj()

        #                 self.dsrtraj_runstat.emit("Stop moving HDI traj. #9...")
                            
        #                 sleep(1)
                            
        #                 self.stop()

        #                 break
                            
        #             # if self.bRunPauseFlag == True:
        #             #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #1...")
        #             #     while self.bRunPauseFlag:
        #             #         if self.bRunPauseFlag is False:
        #             #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #1...")
        #             #             break
                            
        #             #         sleep(0.1)

        #             if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
        #                 break

        #             sleep(0.1)

        #         if self.bRunFlag == True:
        #             print("Complete moving HDI Trjaectory #9...")

        #             self.setuvlamp.emit(True)

        #             print("Waiting during disinfection...")

        #             sleep(DTRunTime)

        #             print("Done first device disinfection...")

        #             self.setuvlamp.emit(False)

        #             sleep(2)

        #             zbdsr_traj = ZBDSRSetTraj()            
        #             zbdsr_traj.trajnum = 1
        #             zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
        #             self.dsrpub.publish(zbdsr_traj)

        #             sleep(2)

        #             self.RunDSRTraj(10)

        #             print("[device3_1] Send topic [Run Trajectory 10]")

        #             self.trajStatus = ZBDSRTrajStatus.IDLING

        #             while True:
        #                 if self.bEStop == True:
        #                     continue

        #                 if self.bEStop == False and self.bprevEStop == True:
        #                     print("====> delay 10 sec...")
        #                     sleep(10)
        #                     self.bprevEStop = False

        #                 if self.bRunFlag == False:
        #                     self.stopTraj()

        #                     self.dsrtraj_runstat.emit("Stop moving HDI traj. #10...")
                                
        #                     sleep(1)
                                
        #                     self.stop()

        #                     break
                                
        #                 # if self.bRunPauseFlag == True:
        #                 #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #2...")
        #                 #     while self.bRunPauseFlag:
        #                 #         if self.bRunPauseFlag is False:
        #                 #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #2...")
        #                 #             break
                                
        #                 #         sleep(0.1)
                    
        #                 if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
        #                     break

        #                 sleep(0.1)

        #             if self.bRunFlag == True:
        #                 print("Complete moving HDI Trjaectory #10...")        

        #                 self.setfwf.emit(False)
        #                 print("Turn off Front WP")

        #                 self.setbwf.emit(False)
        #                 print("Turn off Rear WP")

        #                 sleep(8)

        #                 print("Complete second device disinfection...")

        #     else:
        #         print("Skip second device disinfection...")

        # bErrFlag = False

        # ======================================
        # Hallow Disinfection Process return #3
        # ======================================
        
        # if self.bRunFlag == True:   
        #     self.naviStatus = NavigationControlStatus.IDLING

        #     os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #     self.gotoWayPoint("device3")

        #     sleep(5)

        #     runcnt = 0
            
        #     while True:
        #         if self.bEStop == True:
        #             continue

        #         if self.bRunFlag == False:
        #             self.stopTraj()

        #             self.navi_runstat.emit("Stop ret device3 HDI process...")

        #             self.setrunflag.emit(True)

        #             sleep(1)

        #             self.stop()

        #             bErrFlag = True

        #             break

        #         if self.bRunPauseFlag == True:
        #             self.navi_runstat.emit("Pause ret device3 HDI process...")
        #             while self.bRunPauseFlag:
        #                 if self.bRunPauseFlag is False:
        #                     self.navi_runstat.emit("Restart ret device3 HDI process...")
        #                     break
                    
        #                 sleep(0.1)            
        
        #         if self.naviStatus == NavigationControlStatus.COMPLETED:
        #             #self.HDIRun[2] = 1

        #             print("Arrived at ret device3...")
        #             break

        #         if self.naviStatus == NavigationControlStatus.WARNNRGTO:
        #             runcnt = runcnt + 1

        #             if runcnt <= self.RepCnt:
        #                 os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #                 self.gotoWayPoint("device3")

        #                 self.navi_runstat.emit("Restart ret device3")

        #                 print("Restart ret Device3 : " + str(runcnt))

        #                 sleep(2)

        #             else:
        #                 runcnt = 0

        #                 self.navi_runstat.emit("Error(WARNNRGTO) ret device3")

        #             break

        #         if self.naviStatus == NavigationControlStatus.ERRGTGF:
        #             self.navi_runstat.emit("Error(ERRGTGF) ret device3")

        #         sleep(0.1)


        #     if self.bRunFlag == True:
        #         sleep(2)


        # //////////////////////////////////////////
        #               DEVICE 4
        # //////////////////////////////////////////

        # =================================
        # Hallow Disinfection Process #4
        # =================================
        
        # if (self.HDIRun[3] == '0') and self.bRunFlag == True:   
        #     self.naviStatus = NavigationControlStatus.IDLING

        #     os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #     self.gotoWayPoint("device4")

        #     sleep(5)

        #     runcnt = 0
            
        #     while True:
        #         if self.bEStop == True:
        #             continue

        #         if self.bRunFlag == False:
        #             self.stopTraj()

        #             self.navi_runstat.emit("Stop device4 HDI process...")

        #             self.setrunflag.emit(True)

        #             sleep(1)

        #             self.stop()

        #             bErrFlag = True

        #             break

        #         if self.bRunPauseFlag == True:
        #             self.navi_runstat.emit("Pause device4 HDI process...")
        #             while self.bRunPauseFlag:
        #                 if self.bRunPauseFlag is False:
        #                     self.navi_runstat.emit("Restart device4 HDI process...")
        #                     break
                    
        #                 sleep(0.1)            
        
        #         if self.naviStatus == NavigationControlStatus.COMPLETED:
        #             #self.HDIRun[2] = 1

        #             print("Arrived at device4...")
        #             break

        #         if self.naviStatus == NavigationControlStatus.WARNNRGTO:
        #             runcnt = runcnt + 1

        #             if runcnt <= self.RepCnt:
        #                 os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #                 self.gotoWayPoint("device4")

        #                 self.navi_runstat.emit("Restart device4")

        #                 print("Restart Device4 : " + str(runcnt))

        #                 sleep(2)

        #             else:
        #                 runcnt = 0

        #                 self.navi_runstat.emit("Error(WARNNRGTO) device4")

        #             break

        #         if self.naviStatus == NavigationControlStatus.ERRGTGF:
        #             self.navi_runstat.emit("Error(ERRGTGF) device4")

        #         sleep(0.1)


        #     if self.bRunFlag == True:
        #         sleep(2)



        # =================================
        # Hallow Disinfection Process #4_1
        # =================================

        # if self.HDIRun[3] == '0' and self.bRunFlag == True:   
        #     #self.trajStatus = ZBDSRTrajStatus.IDLING
        #     self.naviStatus = NavigationControlStatus.IDLING

        #     os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #     self.gotoWayPoint("device4_1")

        #     sleep(5)

        #     runcnt = 0
            
        #     while True:
        #         if self.bEStop == True:
        #             continue

        #         if self.bRunFlag == False:
        #             self.StopWaypoint()

        #             self.navi_runstat.emit("Stop device4_1 HDI process...")

        #             self.setrunflag.emit(True)
                    
        #             sleep(1)
                    
        #             self.stop()

        #             bErrFlag = True

        #             break                    
                    
        #         if self.bRunPauseFlag == True:
        #             self.navi_runstat.emit("Pause device4_1 HDI process...")
        #             while self.bRunPauseFlag:
        #                 if self.bRunPauseFlag is False:
        #                     self.navi_runstat.emit("Restart device4_1 HDI process...")
        #                     break
                    
        #                 sleep(0.1)
            
        #         if self.naviStatus == NavigationControlStatus.COMPLETED:
        #             self.HDIRun[3] = 1

        #             print("Arrived at device4_1...")

        #             bErrFlag = False
        #             break

        #         if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
        #             runcnt = runcnt + 1

        #             if runcnt <= self.RepCnt:
        #                 os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #                 self.gotoWayPoint("device4_1")

        #                 self.navi_runstat.emit("Restart device4_1")

        #                 print("Restart device4_1 : " + str(runcnt))

        #                 self.naviStatus = NavigationControlStatus.IDLING

        #                 sleep(2)

        #             else:
        #                 self.HDIRun[3] = 0

        #                 runcnt = 0

        #                 self.navi_runstat.emit("Error(WARNNRGTO) device4_1")

        #                 bErrFlag = True

        #                 break

        #         sleep(0.1)


        #     if bErrFlag == False and self.bRunFlag == True:
        #         self.trajStatus = ZBDSRTrajStatus.IDLING

        #         self.setfwf.emit(True)
        #         print("Turn on Front WP")

        #         self.setbwf.emit(True)
        #         print("Turn on Rear WP")

        #         sleep(5)
        #         # sleep(2)

        #         self.RunDSRTraj(9)

        #         print("[device4_1] Send topic [Run Trajectory 9]")

        #         while True:
        #             if self.bEStop == True:
        #                 continue

        #             if self.bEStop == False and self.bprevEStop == True:
        #                 print("====> delay 10 sec...")
        #                 sleep(10)
        #                 self.bprevEStop = False

        #             if self.bRunFlag == False:
        #                 self.stopTraj()

        #                 self.dsrtraj_runstat.emit("Stop moving HDI traj. #9...")
                            
        #                 sleep(1)
                            
        #                 self.stop()

        #                 break
                            
        #             # if self.bRunPauseFlag == True:
        #             #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #1...")
        #             #     while self.bRunPauseFlag:
        #             #         if self.bRunPauseFlag is False:
        #             #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #1...")
        #             #             break
                            
        #             #         sleep(0.1)

        #             if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
        #                 break

        #             sleep(0.1)

        #         if self.bRunFlag == True:
        #             print("Complete moving HDI Trjaectory #9...")

        #             self.setuvlamp.emit(True)

        #             print("Waiting during disinfection...")

        #             sleep(DTRunTime)

        #             print("Done first device disinfection...")

        #             self.setuvlamp.emit(False)

        #             sleep(2)

        #             zbdsr_traj = ZBDSRSetTraj()            
        #             zbdsr_traj.trajnum = 1
        #             zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
        #             self.dsrpub.publish(zbdsr_traj)

        #             sleep(2)

        #             self.RunDSRTraj(10)

        #             print("[Device4] Send topic [Run Trajectory 10]")

        #             self.trajStatus = ZBDSRTrajStatus.IDLING

        #             while True:
        #                 if self.bEStop == True:
        #                     continue

        #                 if self.bEStop == False and self.bprevEStop == True:
        #                     print("====> delay 10 sec...")
        #                     sleep(10)
        #                     self.bprevEStop = False

        #                 if self.bRunFlag == False:
        #                     self.stopTraj()

        #                     self.dsrtraj_runstat.emit("Stop moving HDI traj. #10...")
                                
        #                     sleep(1)
                                
        #                     self.stop()

        #                     break
                                
        #                 # if self.bRunPauseFlag == True:
        #                 #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #2...")
        #                 #     while self.bRunPauseFlag:
        #                 #         if self.bRunPauseFlag is False:
        #                 #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #2...")
        #                 #             break
                                
        #                 #         sleep(0.1)
                    
        #                 if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
        #                     break

        #                 sleep(0.1)

        #             if self.bRunFlag == True:
        #                 print("Complete moving HDI Trjaectory #10...")        

        #                 self.setfwf.emit(False)
        #                 print("Turn off Front WP")

        #                 self.setbwf.emit(False)
        #                 print("Turn off Rear WP")

        #                 sleep(8)

        #                 print("Complete second device disinfection...")

        #     else:
        #         print("Skip second device disinfection...")

        # bErrFlag = False

        # ======================================
        # Hallow Disinfection Process return #4
        # ======================================
        
        # if self.bRunFlag == True:   
        #     self.naviStatus = NavigationControlStatus.IDLING

        #     os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #     self.gotoWayPoint("device4")

        #     sleep(5)

        #     runcnt = 0
            
        #     while True:
        #         if self.bEStop == True:
        #             continue

        #         if self.bRunFlag == False:
        #             self.stopTraj()

        #             self.navi_runstat.emit("Stop ret device4 HDI process...")

        #             self.setrunflag.emit(True)

        #             sleep(1)

        #             self.stop()

        #             bErrFlag = True

        #             break

        #         if self.bRunPauseFlag == True:
        #             self.navi_runstat.emit("Pause ret device4 HDI process...")
        #             while self.bRunPauseFlag:
        #                 if self.bRunPauseFlag is False:
        #                     self.navi_runstat.emit("Restart ret device4 HDI process...")
        #                     break
                    
        #                 sleep(0.1)            
        
        #         if self.naviStatus == NavigationControlStatus.COMPLETED:
        #             #self.HDIRun[2] = 1

        #             print("Arrived at ret device4...")
        #             break

        #         if self.naviStatus == NavigationControlStatus.WARNNRGTO:
        #             runcnt = runcnt + 1

        #             if runcnt <= self.RepCnt:
        #                 os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #                 self.gotoWayPoint("device4")

        #                 self.navi_runstat.emit("Restart ret device4")

        #                 print("Restart ret Device4 : " + str(runcnt))

        #                 sleep(2)

        #             else:
        #                 runcnt = 0

        #                 self.navi_runstat.emit("Error(WARNNRGTO) ret device4")

        #             break

        #         if self.naviStatus == NavigationControlStatus.ERRGTGF:
        #             self.navi_runstat.emit("Error(ERRGTGF) ret device4")

        #         sleep(0.1)


        #     if self.bRunFlag == True:
        #         sleep(2)


        # ========================================
        # Hallow Disinfection Process lobbypos
        # ========================================
        
        if (self.HDIRun[2] == '0' or self.HDIRun[3] == '0') and self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("lobbypos4")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("lobbypos4")

                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop lobbypos4 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause lobbypos4 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart lobbypos4 HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    #self.HDIRun[4] = 1

                    print("Arrived at lobbypos4...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("lobbypos4")

                        self.navi_runstat.emit("Restart lobbypos4")

                        print("Restart lobbypos4 : " + str(runcnt))

                    else:
                        #self.HDIRun[4] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) lobbypos4")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) lobbypos4")

                    #self.HDIRun[4] = 0

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(2)


        # //////////////////////////////////////////
        #               DEVICE 5
        # //////////////////////////////////////////

        # =================================
        # Hallow Disinfection Process #5
        # =================================
        
        if (self.HDIRun[2] == '0') and self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device5")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device5")

                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop device5 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause device5 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart device5 HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    print("Arrived at device5...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device5")

                        self.navi_runstat.emit("Restart device5")

                        print("Restart device5 : " + str(runcnt))

                        sleep(2)

                    else:
                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) device5")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) device5")

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(2)


        # =================================
        # Hallow Disinfection Process #5_1
        # =================================        

        if self.HDIRun[2] == '0' and self.bRunFlag == True: 
            self.trajStatus = ZBDSRTrajStatus.IDLING
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device5_1")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device5_1")

                if self.bRunFlag == False:
                    self.StopWaypoint()

                    self.navi_runstat.emit("Stop device5_1 HDI process...")

                    self.setrunflag.emit(True)
                    
                    sleep(1)
                    
                    self.stop()

                    bErrFlag = True

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause device5_1 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart device5_1 HDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:

                    self.HDIRun[2] = 1

                    print("Arrived at device5_1...")

                    bErrFlag = False
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device5_1")

                        self.navi_runstat.emit("Restart device5_1")

                        print("Restart device5_1 : " + str(runcnt))

                        self.naviStatus = NavigationControlStatus.IDLING

                        sleep(2)

                    else:
                        self.HDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) device5_1")

                        bErrFlag = True

                        break

                sleep(0.1)


            if bErrFlag == False and self.bRunFlag == True:
                self.setfwf.emit(True)
                print("Turn on Front WP")

                sleep(0.5)

                self.CheckEStop()

                self.setbwf.emit(True)
                print("Turn on Rear WP")

                sleep(2)

                self.CheckEStop()

                # sleep(5)

                self.RunDSRTraj(9)

                print("[device5_1] Send topic [Run Trajectory 9]")

                while True:
                    if self.bEStop == True:
                        continue

                    if self.bEStop == False and self.bprevEStop == True:
                        print("====> delay 5 sec...")
                        sleep(5)
                        self.bprevEStop = False

                    if self.bRunFlag == False:
                        self.stopTraj()
                        
                        self.dsrtraj_runstat.emit("Stop moving HDI traj. #9...")

                        sleep(1)
                        
                        self.stop()

                        break
                        
                    # if self.bRunPauseFlag == True:
                    #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #3...")
                    #     while self.bRunPauseFlag:
                    #         if self.bRunPauseFlag is False:
                    #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #3...")
                    #             break
                        
                    #         sleep(0.1)

                    if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                        break

                    sleep(0.1)

                if self.bRunFlag == True:
                    self.CheckEStop()

                    print("Complete moving HDI Trjaectory #9...")

                    self.setuvlamp.emit(True)

                    print("Waiting during disinfection...")

                    self.CheckEStop()

                    sleep(DTRunTime)

                    print("Done first device disinfection...")

                    self.CheckEStop()

                    self.setuvlamp.emit(False)

                    sleep(2)

                    self.CheckEStop()

                    zbdsr_traj = ZBDSRSetTraj()            
                    zbdsr_traj.trajnum = 9
                    zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                    self.dsrpub.publish(zbdsr_traj)

                    sleep(2)

                    self.CheckEStop()

                    self.RunDSRTraj(10)

                    print("[device5_1] Send topic [Run Trajectory 10]")

                    self.trajStatus = ZBDSRTrajStatus.IDLING

                    while True:
                        if self.bEStop == True:
                            continue

                        if self.bEStop == False and self.bprevEStop == True:
                            print("====> delay 5 sec...")
                            sleep(5)
                            self.bprevEStop = False

                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.dsrtraj_runstat.emit("Stop moving HDI traj. #10...")
                            
                            sleep(1)
                            
                            self.stop()

                            break
                            
                        # if self.bRunPauseFlag == True:
                        #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #4...")
                        #     while self.bRunPauseFlag:
                        #         if self.bRunPauseFlag is False:
                        #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #4...")
                        #             break
                            
                        #         sleep(0.1)

                        if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                            break

                        sleep(0.1)

                    if self.bRunFlag == True:
                        print("Complete moving HDI Trjaectory #10...")      

                        self.CheckEStop()

                        self.setfwf.emit(False)
                        print("Turn off Front WP")

                        sleep(0.5)

                        self.CheckEStop()

                        self.setbwf.emit(False)
                        print("Turn off Rear WP")

                        sleep(5)  

                        print("Complete third device disinfection...")

            else:
                print("Skip third device disinfection...")

        bErrFlag = False


        # ======================================
        # Hallow Disinfection Process return #5
        # ======================================

        if self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device5")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device5")

                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop ret device5 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause ret device5 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart ret device5 HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    print("Arrived at ret device5...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device5")

                        self.navi_runstat.emit("Restart ret device5")

                        print("Restart ret device5 : " + str(runcnt))

                        sleep(2)

                    else:
                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) ret device5")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) ret device5")

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(2)


        # //////////////////////////////////////////
        #               DEVICE 6
        # //////////////////////////////////////////

        # =================================
        # Hallow Disinfection Process #6
        # =================================
        
        if (self.HDIRun[3] == '0') and self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device6")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device6")

                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop device6 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause device6 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart device6 HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    print("Arrived at device6...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device6")

                        self.navi_runstat.emit("Restart device6")

                        print("Restart device6 : " + str(runcnt))

                        sleep(2)

                    else:
                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) device6")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) device6")

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(2)


        # =================================
        # Hallow Disinfection Process #6_1
        # =================================

        if self.HDIRun[3] == '0' and self.bRunFlag == True: 
            self.trajStatus = ZBDSRTrajStatus.IDLING
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device6_1")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device6_1")

                if self.bRunFlag == False:
                    self.StopWaypoint()

                    self.navi_runstat.emit("Stop device6_1 HDI process...")

                    self.setrunflag.emit(True)
                    
                    sleep(1)
                    
                    self.stop()

                    bErrFlag = True

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause device6_1 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart device6_1 HDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.HDIRun[3] = 1

                    print("Arrived at device6_1...")
                    bErrFlag = False
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device6_1")

                        self.navi_runstat.emit("Restart device6_1")

                        print("Restart device6_1 : " + str(runcnt))

                        self.naviStatus = NavigationControlStatus.IDLING

                        sleep(2)

                    else:
                        self.HDIRun[3] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) device6_1")

                        bErrFlag = True

                        break

                sleep(0.1)

            if bErrFlag == False and self.bRunFlag == True:

                self.setfwf.emit(True)
                print("Turn on Front WP")

                sleep(0.5)

                self.CheckEStop()

                self.setbwf.emit(True)
                print("Turn on Rear WP")

                sleep(2.0)
                #sleep(5)

                self.CheckEStop()

                self.RunDSRTraj(9)

                print("[Device6] Send topic [Run Trajectory 9]")

                while True:
                    if self.bEStop == True:
                        continue

                    if self.bEStop == False and self.bprevEStop == True:
                        print("====> delay 5 sec...")
                        sleep(5)
                        self.bprevEStop = False

                    if self.bRunFlag == False:
                        self.stopTraj()

                        self.dsrtraj_runstat.emit("Stop moving HDI traj. #9...")
                        
                        sleep(1)
                        
                        self.stop()

                        break
                        
                    # if self.bRunPauseFlag == True:
                    #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #3...")
                    #     while self.bRunPauseFlag:
                    #         if self.bRunPauseFlag is False:
                    #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #3...")
                    #             break
                        
                    #         sleep(0.1)
                
                    if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                        break

                    sleep(0.1)

                
                if self.bRunFlag == True:
                    print("Complete moving HDI Trjaectory #9...")

                    self.CheckEStop()

                    self.setuvlamp.emit(True)

                    print("Waiting during disinfection...")

                    self.CheckEStop()

                    sleep(DTRunTime)

                    self.CheckEStop()

                    print("Done first device disinfection...")

                    self.setuvlamp.emit(False)

                    sleep(2)

                    self.CheckEStop()

                    zbdsr_traj = ZBDSRSetTraj()            
                    zbdsr_traj.trajnum = 9
                    zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                    self.dsrpub.publish(zbdsr_traj)

                    sleep(2)

                    self.CheckEStop()

                    self.RunDSRTraj(10)

                    print("[Device6] Send topic [Run Trajectory 10]")

                    self.trajStatus = ZBDSRTrajStatus.IDLING

                    while True:
                        if self.bEStop == True:
                            continue

                        if self.bEStop == False and self.bprevEStop == True:
                            print("====> delay 5 sec...")
                            sleep(5)
                            self.bprevEStop = False

                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.dsrtraj_runstat.emit("Stop moving HDI traj. #10...")
                            
                            sleep(1)
                            
                            self.stop()

                            break
                            
                        # if self.bRunPauseFlag == True:
                        #     self.dsrtraj_runstat.emit("Pause moving HDI traj. #4...")
                        #     while self.bRunPauseFlag:
                        #         if self.bRunPauseFlag is False:
                        #             self.dsrtraj_runstat.emit("Restart moving HDI traj. #4...")
                        #             break
                            
                        #         sleep(0.1)

                        if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                            break

                        sleep(0.1)

                    if self.bRunFlag == True:
                        print("Complete moving HDI Trjaectory #10...")   

                        self.CheckEStop()

                        self.setfwf.emit(False)
                        print("Turn off Front WP")

                        sleep(0.5)

                        self.CheckEStop()

                        self.setbwf.emit(False)
                        print("Turn off Rear WP")

                        sleep(5)  

            else:
                print("Skip moving HDI Trjaectory #10...")   

        # ======================================
        # Hallow Disinfection Process return #6
        # ======================================
 
        if self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device6")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bEStop == True:
                    continue

                self.CheckAborted("device6")

                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop ret device6 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause ret device6 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart ret device6 HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    print("Arrived at ret device6...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device6")

                        self.navi_runstat.emit("Restart ret device6")

                        print("Restart ret device6 : " + str(runcnt))

                        sleep(2)

                    else:
                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) ret device6")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) ret device6")

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(2)



    def gotoChargingStation(self):
        # =================================
        # Hallow Disinfection Process #11
        # =================================
        
        self.naviStatus = NavigationControlStatus.IDLING

        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        self.gotoWayPoint("chargingpos")

        sleep(5)

        runcnt = 0
        
        while True:      
            if self.bEStop == True:
                continue

            self.CheckAborted("chargingpos")

            if self.bRunFlag == False:
                self.StopWaypoint()

                self.navi_runstat.emit("Stop chargingpos HDI process...")

                self.setrunflag.emit(True)
                
                sleep(1)
                
                self.stop()

                break
                
            if self.bRunPauseFlag == True:
                self.navi_runstat.emit("Pause chargingpos HDI process...")
                while self.bRunPauseFlag:
                    if self.bRunPauseFlag is False:
                        self.navi_runstat.emit("Restart chargingpos HDI process...")
                        break
                
                    sleep(0.1)
        
            if self.naviStatus == NavigationControlStatus.COMPLETED:
                
                #self.HDIRun[6] = 1

                print("Arrived at chargingpos...")
                break

            if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                runcnt = runcnt + 1

                if runcnt < self.RepCnt:
                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                    self.gotoWayPoint("chargingpos")

                    self.navi_runstat.emit("Restart chargingpos")

                    print("Restart chargingpos : " + str(runcnt))

                    sleep(2)

                else:

                    runcnt = 0

                    self.navi_runstat.emit("Error(WARNNRGTO) chargingpos")

                break

            sleep(0.1)

        if self.bRunFlag == True:

            print("Arrived at charging station position...")  
            
            # 2022.03.02
            self.setlaser.emit(False)

            client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            client.wait_for_server()

            goal = ChargingGoal()
            client.send_goal(goal)

            self.bCSParking = True

            print("Parking charging station...")

            while True:
                if self.bEStop == True:
                    continue

                if self.bRunFlag == False:
                    cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
                    os.system(cmdstr)

                    self.navi_runstat.emit("Stop auto charging process...")
                    
                    sleep(1)
                    
                    self.stop()
                    
                    break

                if self.csStatus == "contact":
                    print("====>csStatus : " + self.csStatus)
                    self.navi_runstat.emit("Complete parking charging station.")

                    # 2022. 03. 03
                    # self.setapflag.emit(True)
                    self.setrunflag.emit(False)  
                    
                    # 2022. 03. 03
                    self.setpcsbtn.emit(False)                 
                    
                    break
                    
                sleep(0.1)
    

    def run(self):

        for i in range(1, 15):
            self.HDIRun[i] = 0

        if self.bRunFlag == True:
            self.navi_Process()

            print("First time : Complete forth device disinfection...")
                
            # if self.HDIRun[0] == '0' or self.HDIRun[1] == '0' or self.HDIRun[2] == '0' or self.HDIRun[3] == '0' or self.HDIRun[4] == '0' :
            # #if self.HDIRun[0] == '0' or self.HDIRun[1] == '0' or self.HDIRun[2] == '0' or self.HDIRun[3] == '0' or self.HDIRun[4] == '0' or self.HDIRun[5] == '0' or self.HDIRun[6] == '0' or self.HDIRun[7] == '0':

            #    self.navi_Process()

            print("Second time : Complete forth device disinfection...")
        
            print("Complete forth device disinfection...")

            print("Go to charging station....")

            # self.setlaser.emit(False)

            # onRedLaserCtrlBtn()
            # onGreenLaserCtrlBtn()
        
            self.gotoChargingStation()

            print("Complete all disinfection processes in the hallow disindection.")

            self.bRunFlag = False

        self.stop()
  

    
    def stop(self):
        self.quit()
        self.wait(1000)
        


class Thread_TeleOP(QThread):
    print_lspeed = pyqtSignal(str)
    print_aspeed = pyqtSignal(str)
    print_lsinc = pyqtSignal(str)
    print_asinc = pyqtSignal(str)

    def __init__(self, parent = None):
        super(Thread_TeleOP, self).__init__(parent)
        #self.working = True
        #self.n = 0
        #self.main = parent
        #self.isRun = False

        self.count = 0
        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.speed = initSpeed
        self.turn = initTurn
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

    def initVal(self):
        self.count = 0
        self.x = 0.0
        self.th = 0.0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.speed = initSpeed
        self.turn = initTurn
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        self.print_lsinc.emit('%.2f' % (self.speed))
        self.print_asinc.emit('%.2f' % (self.turn))
        self.print_lspeed.emit('%.2f' % (self.control_speed))
        self.print_aspeed.emit('%.2f' % (self.control_turn))

    def run(self):
        global convkey
        global prev_convkey
        global bRunMCtrl

        while bRunMCtrl == True:
            if convkey in moveBindings.keys():
                try:
                    self.x = moveBindings[convkey][0]
                    self.th = moveBindings[convkey][1]
                except:
                    pass

                # #self.count = 0
                # print vels(self.control_speed, self.control_turn)
                # # prev_convkey = convkey

            # elif convkey in speedBindings.keys():
            #     self.speed = self.speed * speedBindings[convkey][0]              
            #     self.turn = self.turn * speedBindings[convkey][1]
            #     #self.count = 0
                
            #     print vels(self.speed, self.turn)
            #     # prev_convkey = convkey

            elif convkey == 's' :
                self.x = 0
                self.th = 0
                self.control_speed = 0
                self.control_turn = 0

            elif convkey == 'r':
                if prev_convkey in speedBindings.keys():
                    try:
                        #print("[2] speed:" + str(self.speed) + " turn :" + str(self.turn))

                        self.speed = self.speed * speedBindings[prev_convkey][0]              
                        self.turn = self.turn * speedBindings[prev_convkey][1]

                        #print("[3] speed:" + str(self.speed) + " turn :" + str(self.turn))

                        
                    except:
                        pass
                    else:    
                        #mstr = str(self.speed)                        
                        self.print_lsinc.emit('%.2f' % (self.speed))
                        #self.print_lsinc.emit(f'{self.speed:.2f}')
                        #self.print_lsinc.emit(str(self.speed))
                        #mstr = str(self.turn)
                        self.print_asinc.emit('%.2f' % (self.turn))
                        #self.print_asinc.emit(f'{self.turn:.2f}')
                        #self.print_asinc.emit(str(self.turn))

                        convkey = 0
                        #print vels(self.speed, self.turn)

                self.x = 0
                self.th = 0

                # prev_convkey = convkey

            # sleep(0.1)

            # if prev_convkey != convkey:
            #     self.x = 0
            #     self.th = 0

            # else:
            #     self.count = self.count + 1
            #     if self.count > 4:
            #         self.x = 0
            #         self.th = 0
                    
            
            self.target_speed = self.speed * self.x

            if self.target_speed >= 1.0:
                self.target_speed = 1.0
            self.target_turn = self.turn * self.th
            if self.target_turn>= 1.0:
                self.target_turn = 1.0

            if self.target_speed > self.control_speed:
                self.control_speed = min( self.target_speed, self.control_speed + 0.02 )
            elif self.target_speed < self.control_speed:
                self.control_speed = max( self.target_speed, self.control_speed - 0.02 )
            else:
                self.control_speed = self.target_speed

            if self.target_turn > self.control_turn:
                self.control_turn = min( self.target_turn, self.control_turn + 0.1 )
            elif self.target_turn < self.control_turn:
                self.control_turn = max( self.target_turn, self.control_turn - 0.1 )
            else:
                self.control_turn = self.target_turn

            self.print_lspeed.emit('%.2f' % (self.control_speed))
            self.print_aspeed.emit('%.2f' % (self.control_turn))

            twist = Twist()
            twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
            self.pub_twist.publish(twist)

        print("Exit thread of TeleOP...")

        
class MyWindow(QMainWindow, form_class):
    def __init__(self):
        super(QMainWindow, self).__init__()
        self.setupUi(self)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

        self.settings = termios.tcgetattr(sys.stdin)

        self.bFrontWF = False
        self.bRearWF = False
        self.bFrontSWF = False
        self.bRearSWF = False
        self.bCRDIFrontSWF = False
        self.bCRDIRearSWF = False

        self.SelectWF = 0b00
        self.SelWFRange = 0b11
        self.SelectCRDIWF = 0b00
        self.SelCRDIWFRange = 0b11

        self.bBringUpRun = False
        self.bSLAMRun = False
        self.bTeleOPRun = False
        self.b2LRFRun = False
        self.bNaviRun = False
        self.bRVIZRun = False
        self.bWPCRun = False
        self.bMakeWPRun = False
        self.bDSRRobotRun = False
        self.bTraj1Run = False
        self.bDSRStart = False

        self.bStopStart = False
        self.bReadyRun = False
        self.dsrServiceConOK = False
        self.cur_trajnum = 0

        self.bCSParking = False

        self.bmotionCtrl = False
        #self.bRunMCtrl = False

        self.incdec_val = 0.03
        self.status = 0
        self.target_linear_vel = 0
        self.target_angular_vel = 0
        self.control_linear_vel = 0
        self.control_angular_vel = 0

        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.speed = 0.1
        self.turn = 0.2
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        self.naviStatus = NavigationControlStatus.IDLING
        self.bHDIRun = False
        self.bCRDIRun = False

        self.bAutoPark = False
        self.bHDIAutoPark = False
        self.bCSDIAutoPark = False

        self.bStopFlag = False
        self.bPauseFlag = False

        self.bGreenLaserOn = True
        self.bRedLaserOn = True

        self.bStartOK = False
        
        self.IMUVCnt = 0

        self.bSLAMRVIZRun = False

        try :
            with open(yaml_fname) as f:
                wp_list = yaml.safe_load(f)
                print(wp_list)
                
                wps = wp_list['waypoints']
                print("waypoints")
                for n in wps:
                    print("name : " + n["name"])
                    self.wp_CB.addItem(n["name"])

                print("trajectories")
                trajs = wp_list['trajectories']
                for t in trajs:
                    print("name : " + t["name"])
                    self.traj_CB.addItem(t["name"])

            for i in range(1, 11):
                trajname =  "Trajectory {0}".format(i)
                #print(trajname)
                self.dsrtraj_CB.addItem(trajname)
        except :
            bHDIWPFileLoadOK = False
            
        finally:
            bHDIWPFileLoadOK = True
        
        self.messLV = QStandardItemModel()

        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)

        self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRSetTraj, queue_size = 10)

        rospy.Subscriber("/status_sdr_traj", ZBDSRTrajStatus, self.CallbackRunTrajStatus)

        rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)

        rospy.Subscriber("/imu", Imu, self.recv_IMU)

        rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)

        
        
        self.wpCBStr = self.wp_CB.currentText()
        self.trajCBStr = self.traj_CB.currentText()
        self.dsrtrajCBStr = self.dsrtraj_CB.currentText()
        self.wp_CB.activated[str].connect(self.onActivatedwpCB)
        self.traj_CB.activated[str].connect(self.onActivatedtrajCB)
        self.dsrtraj_CB.activated[str].connect(self.onActivateddsrtrajCB)

        self.print_MessStr("Compelte initialization...")

        self.wp_startPB.clicked.connect(self.onWPStartBtn)
        self.wp_stopPB.clicked.connect(self.onWPStopBtn)
        # self.wp_pausePB.clicked.connect(self.onWPPauseBtn)
        self.dsrtraj_runPB.clicked.connect(self.onDSRTrajRunBtn)
        self.dsrtraj_stopstartPB.clicked.connect(self.onDSRTrajStopStartBtn)
        self.traj_startPB.clicked.connect(self.onTrajStartBtn)
        self.traj_stopPB.clicked.connect(self.onTrajStopBtn)
        # self.traj_pausePB.clicked.connect(self.onTrajPauseBtn)
        self.run_roscorePB.clicked.connect(self.onROSCoreBtn)
        self.run_bringupPB.clicked.connect(self.onBringupBtn)
        self.run_slamPB.clicked.connect(self.onSlamBtn)
        self.run_teleopPB.clicked.connect(self.onTeleopBtn)
        self.run_2LRFPB.clicked.connect(self.on2LFRBtn)
        self.run_naviPB.clicked.connect(self.onNavigationBtn)
        self.run_rvizPB.clicked.connect(self.onRVIZBtn)
        
        self.run_slamrvizPB.clicked.connect(self.onSLAMRVIZBtn)
        
        self.run_wpcPB.clicked.connect(self.onWPCBtn)
        self.run_makeWPPB.clicked.connect(self.onMakeWPBtn)
        self.frontwfcPB.clicked.connect(self.onFrontWFCtrlPBtn)
        self.rearwfcPB.clicked.connect(self.onRearWFCtrlPBtn)       
        self.frontselwfrnagePB.clicked.connect(self.onFrontSelWFRangePBtn)
        self.rearselwfrnagePB.clicked.connect(self.onRearSelWFRangePBtn)
        self.pumpOnPB.clicked.connect(self.onPumpOnPBtn)
        self.pumpOffPB.clicked.connect(self.onPumpOffPBtn)
        self.solOnPB.clicked.connect(self.onSolOnPBtn)
        self.solOffPB.clicked.connect(self.onSolOffPBtn)
        self.run_saveWPPB.clicked.connect(self.onSaveWPBtn)
        self.run_dsrRobotPB.clicked.connect(self.onDSRRobotBtn)
        self.run_Traj1PB.clicked.connect(self.onTraj1Btn)
        self.run_connectSerial1PB.clicked.connect(self.onCoonectSerial1Btn)
        self.run_connectSerial2PB.clicked.connect(self.onCoonectSerial2Btn)
        self.uvcETOnPB.clicked.connect(self.onUVCEndtoolOnBtn)
        self.uvcETOffPB.clicked.connect(self.onUVCEndtoolOffBtn)
        self.uvcBTOnPB.clicked.connect(self.onUVCBottomOnBtn)
        self.uvcBTOffPB.clicked.connect(self.onUVCBottomOffBtn)
        self.scrubCtrlOnPB.clicked.connect(self.onScrubberCtrlOnBtn)
        self.scrubCtrlOffPB.clicked.connect(self.onScrubberCtrlOffBtn)
        self.exitPB.clicked.connect(self.onExit)
        self.initpos_dsrRobotPB.clicked.connect(self.onInitPosDSRRobotBtn)
        self.zeropos_dsrRobotPB.clicked.connect(self.onZeroPosDSRRobotBtn)
        self.csparking_PB.clicked.connect(self.onParkingChargingStationBtn)
        self.motionctrl_PB.clicked.connect(self.onMotionCtrlBtn)
        self.savemapPB.clicked.connect(self.onSaveMapBtn)
        #self.run_navitestPB.clicked.connect(self.onNaviTestBtn)
        self.readyPB.clicked.connect(self.onReadyBtn)
        self.runhdiPB.clicked.connect(self.onHDIBtn)
        self.runcrdiPB.clicked.connect(self.onCRDIBtn)
        self.autoparkingPB.clicked.connect(self.onAutoParkingBtn)
        self.stopprocPB.clicked.connect(self.onStopProcBtn)
        self.pauseprocPB.clicked.connect(self.onPauseProcBtn)

        self.greenLaserCtrlPB.clicked.connect(self.onGreenLaserCtrlBtn)
        self.redLaserCtrlPB.clicked.connect(self.onRedLaserCtrlBtn)

        self.mcinit_PB.clicked.connect(self.InitMCValueBtn)
        
        self.greenLaserCtrlPB.setText('G-Laser Off')
        self.redLaserCtrlPB.setText('R-Laser Off')

        self.lspeed_lineEdit.setText("0.0")
        self.aspeed_lineEdit.setText("0.0")
        self.lsinc_lineEdit.setText(str(initSpeed))
        self.asinc_lineEdit.setText(str(initTurn))

        self.trajmode = NavigationControl.NONE
        self.trajmode_checkBox.stateChanged.connect(self.changeTrajMode)

        # run 12 terminal window
        os.system("python3 ~/bin/target_term -set 13")
        self.print_MessStr("Run terminal windows.")

        data = os.environ["HOME"]+"/.term_list"
        self.t_term = open(data).read().splitlines()
        print(self.t_term)

        self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))       
        self.pumpled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.solled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.UVCBled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        # self.UVCBled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.Scrubled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.glaserled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        self.rlaserled_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))
        

        self.fran_lineEdit.setText("0")
        self.rran_lineEdit.setText("0")

        self.naviHDI_th = Thread_HollowDisfection(self)
        self.naviCRDI_th = Thread_ConfRoomDisfection(self)
        
        self.bStartOK = True

        # self.onUVCBottomOffBtn()


    # @pyqtSlot(int)
    # def getcurtraj(self, trajnum):
    #     trajnum = self.cur_trajnum

    @pyqtSlot(bool)
    def setLaserAct(self, stat):
        if stat == True:
            self.bGreenLaserOn = True
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 2 'true' -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn on the green laser.")
            self.greenLaserCtrlPB.setText('G-Laser Off')
            self.glaserled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            self.bRedLaserOn = True
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 9 'true' -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn on the red laser.")
            self.redLaserCtrlPB.setText('R-Laser Off')
            self.rlaserled_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))

        else:
            self.bGreenLaserOn = False
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 2 'false' -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn off the green laser.")
            self.greenLaserCtrlPB.setText('G-Laser On')
            self.glaserled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            self.bRedLaserOn = False
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 9 'false' -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn off the red laser.")
            self.redLaserCtrlPB.setText('R-Laser On')
            self.rlaserled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    @pyqtSlot(bool)
    def setPCSButton(self, val):
        if val == True:
            self.csparking_PB.setText('Parking C.S.')
        else:
            self.csparking_PB.setText('PCS Cancel')
        
        self.bCSParking = True
        
    @pyqtSlot(str)
    def printLinearSpeed(self, str):
        self.lspeed_lineEdit.setText(str)

    @pyqtSlot(str)
    def printAngleSpeed(self, str):
        self.aspeed_lineEdit.setText(str)

    @pyqtSlot(str)
    def printLSInc(self, str):
        self.lsinc_lineEdit.setText(str)

    @pyqtSlot(str)
    def printASInc(self, str):
        self.asinc_lineEdit.setText(str)

    @pyqtSlot(str)
    def hdi_naviRunState(self, str):
        self.mrstat_lineEdit.setText(str)
        
    @pyqtSlot(str)
    def hdi_trajRunState(self, str):
        self.dsrstat_lineEdit.setText(str)
        
    @pyqtSlot(str)
    def crdi_naviRunState(self, str):
        self.mrstat_lineEdit.setText(str)
        
    @pyqtSlot(str)
    def crdi_trajRunState(self, str):
        self.dsrstat_lineEdit.setText(str)

    @pyqtSlot(bool)
    def setHDIRunFlag(self, val):
        self.bHDIRun = val
        self.print_MessStr("Stop HDI process...")

    @pyqtSlot(bool)
    def setCSDIRunFlag(self, val):
        self.bCRDIRun = val
        self.print_MessStr("Stop CSDI process...")

    @pyqtSlot(bool)
    def setHDIAutoParkingFlag(self, val):
        self.bHDIAutoPark = val
        print("Result of HDI Auto Parking : " + str(val))

    @pyqtSlot(bool)
    def setCSDIAutoParkingFlag(self, val):
        self.bCSDIAutoPark = val
        print("Result of CSDI Auto Parking : " + str(val))

    @pyqtSlot(bool)
    def setFrontWF(self, val):
        if val == True:
            self.bFrontWF = True
            self.SelectWF = ((self.SelectWF & 0b11) | 0b01)

            print(self.SelectWF)

            self.frontwfcPB.setText("Enable Front WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Activate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            #self.onFrontWFCtrlPBtn()
            #print("On Front WP")
        else:
            self.bFrontWF = False
            self.SelectWF = ((self.SelectWF & 0b11) & 0b10)
            
            print(self.SelectWF)

            self.frontwfcPB.setText("Disable Front WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Deactivate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            #self.onFrontWFCtrlPBtn()
            #print("Off Front WP")


    @pyqtSlot(bool)
    def setRearWF(self, val):
        if val == True:
            self.bRearWF = True
            self.SelectWF = ((self.SelectWF & 0b11) | 0b10)
            print(self.SelectWF)

            self.rearwfcPB.setText("Enable Rear WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Activate warning field of rear lidar.")
            self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            #self.onRearWFCtrlPBtn()
            #print("On Rear WP")
        else:
            self.bRearWF = False
            self.SelectWF = ((self.SelectWF & 0b11) & 0b01)
            print(self.SelectWF)

            self.rearwfcPB.setText("Disable Rear WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Deactivate warning field of rear lidar.")
            self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            #self.onRearWFCtrlPBtn()
            #print("Off Rear WP")


    @pyqtSlot(bool)
    def setCRDIFrontWF(self, val):
        if val == True:
            self.bCRDIFrontWF = True
            self.SelectCRDIWF = ((self.SelectWF & 0b11) | 0b01)

            print(self.SelectCRDIWF)

            self.frontwfcPB.setText("Enable Front WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectCRDIWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Activate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            #self.onFrontWFCtrlPBtn()
            #print("On Front WP")
        else:
            self.bCRDIFrontWF = False
            self.SelectCRDIWF = ((self.SelectCRDIWF & 0b11) & 0b10)
            
            print(self.SelectCRDIWF)

            self.frontwfcPB.setText("Disable Front WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectCRDIWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Deactivate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            #self.onFrontWFCtrlPBtn()
            #print("Off Front WP")


    @pyqtSlot(bool)
    def setCRDIRearWF(self, val):
        if val == True:
            self.bCRDIRearWF = True
            self.SelectCRDIWF = ((self.SelectCRDIWF & 0b11) | 0b10)
            print(self.SelectCRDIWF)

            self.rearwfcPB.setText("Enable Rear WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectCRDIWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Activate warning field of rear lidar.")
            self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            #self.onRearWFCtrlPBtn()
            #print("On Rear WP")
        else:
            self.bCRDIRearWF = False
            self.SelectCRDIWF = ((self.SelectCRDIWF & 0b11) & 0b01)
            print(self.SelectCRDIWF)

            self.rearwfcPB.setText("Disable Rear WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectCRDIWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Deactivate warning field of rear lidar.")
            self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            #self.onRearWFCtrlPBtn()
            #print("Off Rear WP")

    @pyqtSlot(bool)
    def setUVLamp(self, val):
        if val == True:
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 15 'true' -1 "
            # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: true\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn on the UVC of end effector.")
            self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))
        else:
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 15 'false' -1 "
            # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: false\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn on the UVC of end effector.")
            self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    @pyqtSlot(bool)
    def setBtUVLamp(self, val):
        if val == True:
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 0 'true' -1 "
            # cmdstr = "gnome-terminal -- rostopic pub /uvc_bottom std_msgs/Bool \"data: true\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn on the UVC of bottom.")
            self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))
        else:
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 0 'false' -1 "
            # cmdstr = "gnome-terminal -- rostopic pub /uvc_bottom std_msgs/Bool \"data: false\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn off the UVC of bottom.")
            self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def CallbackCSStatus(self, request):
        if self.bCSParking == True:
            self.csStatus = request.data

            mstr = "Charging Station Status : " + self.csStatus

            self.mrstat_lineEdit.setText(mstr)

    def changeTrajMode(self, state):
        if state == Qt.Checked:
            self.trajmode = NavigationControl.LOOP
            self.print_MessStr("Navi Waypoint Mode : Loop")
        else:
            self.trajmode = NavigationControl.GOAL
            self.print_MessStr("Navi Waypoint Mode : Goal")


    def onSLAMRVIZBtn(self):
        if self.bSLAMRVIZRun is False:
            self.bSLAMRVIZRun = True
            
            cmdstr = "python3 ~/bin/target_term -run 13 rosrun rviz rviz -d " + slamrviz_fname
            os.system(cmdstr)
            
            self.print_MessStr("Run rosrun SLAM rviz")
            self.run_slamrvizPB.setText('Stop SLAMRVIZ')

            #self.runSaveMap_PB.setEnabled(True)
        else:
            self.bSLAMRVIZRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill $(rosnode list | grep rviz_*)")
            self.print_MessStr("Stop rosrun SLAM rviz")
            self.run_slamrvizPB.setText('Run SLAMRVIZ')

            #self.runSaveMap_PB.setDisabled(True)
            
            
    def InitMCValueBtn(self):
        global mctrl_th

        mctrl_th.initVal()
        
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians

    def recv_IMU(self, msgs):

        if self.bStartOK == False:
            return

        self.IMUVCnt += 1

        if self.IMUVCnt >= 40:
            self.IMUVCnt = 0

            rollx, pitchy, yawz = self.euler_from_quaternion(msgs.orientation.x, msgs.orientation.y, msgs.orientation.z, msgs.orientation.w)

            self.imu_lcdNumber.display(yawz*MATH_RAD2DEG)


    def onGreenLaserCtrlBtn(self):
        if self.bGreenLaserOn == False:
            self.bGreenLaserOn = True
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 2 'true' -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn on the green laser.")
            self.greenLaserCtrlPB.setText('G-Laser Off')
            self.glaserled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        else:
            self.bGreenLaserOn = False
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 2 'false' -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn off the green laser.")
            self.greenLaserCtrlPB.setText('G-Laser On')
            self.glaserled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))


    def onRedLaserCtrlBtn(self):
        if self.bRedLaserOn == False:
            self.bRedLaserOn = True
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 9 'true' -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn on the red laser.")
            self.redLaserCtrlPB.setText('R-Laser Off')
            self.rlaserled_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))
        else:
            self.bRedLaserOn = False
            cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 9 'false' -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Turn off the red laser.")
            self.redLaserCtrlPB.setText('R-Laser On')
            self.rlaserled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))


    def onPauseProcBtn(self):
        if self.bPauseFlag == False:
            self.bPauseFlag = True
            self.pauseprocPB.setText('Restart')
            if self.bHDIRun == True:
                self.naviHDI_th.setPauseFlag(True)
            elif self.bCRDIRun == True:
                self.naviCRDI_th.setPauseFlag(True)
        else:
            self.bPauseFlag = False
            self.pauseprocPB.setText('Pause P.')
            if self.bHDIRun == True:
                self.naviHDI_th.setPauseFlag(False)
            elif self.bCRDIRun == True:
                self.naviCRDI_th.setPauseFlag(False)

    def onStopProcBtn(self):
        if self.bStopFlag == False:
            self.bStopFlag = True
            if self.bHDIRun == True:
                self.naviHDI_th.setRunFlag(False)
                self.bHDIRun = False
            elif self.bCRDIRun == True:
                self.naviCRDI_th.setRunFlag(False)
                self.bCRDIRun = False

    def onAutoParkingBtn(self):
    
        return
        
        autopark_th = Thread_ReturnToCharingStation(self)

        if self.bAutoPark == False:
            self.bAutoPark = True
            autopark_th.setRunType(1)
            self.print_MessStr("Send auto parking command.")
            self.autoparkingPB.setText('Cancel')
        else:
            self.bAutoPark = False
            autopark_th.setRunType(0)
            self.print_MessStr("Send cancel parking command.")
            self.autoparkingPB.setText('Auto C.S.')

        autopark_th.start()

        
    def onCRDIBtn(self):
        if self.bCSDIAutoPark == True:
            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)
            self.print_MessStr("Cancel parking charge station.")
            self.csparking_PB.setText('Parking C.S.')

            sleep(5)

            twist = Twist()
            twist.linear.x = 0.04; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(16)

            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            self.bCSDIAutoPark = False

        if self.bCRDIRun == False:
            self.bCRDIRun = True
        
            #self.naviCRDI_th = Thread_ConfRoomDisfection(self)

            self.naviCRDI_th.setRunFlag(True)
            self.naviCRDI_th.start()
            
            self.naviCRDI_th.navi_runstat.connect(self.crdi_naviRunState)
            self.naviCRDI_th.dsrtraj_runstat.connect(self.crdi_trajRunState)
            self.naviCRDI_th.setrunflag.connect(self.setCSDIRunFlag)
            self.naviCRDI_th.setapflag.connect(self.setCSDIAutoParkingFlag)

            self.naviCRDI_th.setfwf.connect(self.setCRDIFrontWF)
            self.naviCRDI_th.setbwf.connect(self.setCRDIRearWF)
            self.naviCRDI_th.setbtuvclamp.connect(self.setBtUVLamp)
            
            self.naviCRDI_th.setpcsbtn.connect(self.setPCSButton)
            
            

            self.print_MessStr("Run navi Conference Room Disinfection Process thread.")
        
        
    def onHDIBtn(self):

        if self.bHDIAutoPark == True:
            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)
            self.print_MessStr("Cancel parking charge station.")
            self.csparking_PB.setText('Parking C.S.')

            sleep(5)

            twist = Twist()
            twist.linear.x = 0.04; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(16)

            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            self.bHDIAutoPark = False

        if self.bHDIRun == False:
            self.bHDIRun = True
            
            self.naviHDI_th.setRunFlag(True)
            self.naviHDI_th.start()

            self.naviHDI_th.navi_runstat.connect(self.hdi_naviRunState)
            self.naviHDI_th.dsrtraj_runstat.connect(self.hdi_trajRunState)
            self.naviHDI_th.setrunflag.connect(self.setHDIRunFlag)
            self.naviHDI_th.setapflag.connect(self.setHDIAutoParkingFlag)

            self.naviHDI_th.setfwf.connect(self.setFrontWF)
            self.naviHDI_th.setbwf.connect(self.setRearWF)
            self.naviHDI_th.setuvlamp.connect(self.setUVLamp)

            self.naviHDI_th.setlaser.connect(self.setLaserAct)
            

            # self.naviHDI_th.getcurtraj.connect(self.cur_trajnum)            
                        
            self.print_MessStr("Run navi Holley Disinfection Process thread.")


    def onReadyBtn(self):
        if self.bReadyRun == False:
            self.bReadyRun = True

            self.onDSRRobotBtn()
            #sleep(5)

            self.onBringupBtn()
            sleep(6)
            
            self.onTraj1Btn()
            sleep(5)

            self.on2LFRBtn()
            #sleep(2)

            self.onInitPosDSRRobotBtn()
            sleep(5)

            self.onNavigationBtn()
            sleep(6)
            
            self.onRVIZBtn()
            sleep(5)
            
            self.onWPCBtn()
            sleep(5)
          
            self.readyPB.setText('Init')

            self.print_MessStr("Ready to initialize...")

        else:
            self.bReadyRun = False

            self.onInitPosDSRRobotBtn()
            sleep(3)

            self.onWPCBtn()
            sleep(3)

            self.onTraj1Btn()
            sleep(3)

            self.on2LFRBtn()
            sleep(3)

            self.onNavigationBtn()
            sleep(3)

            self.onDSRRobotBtn()
            sleep(3)

            self.onBringupBtn()
            sleep(8)

            self.onRVIZBtn()
            sleep(3)

            self.readyPB.setText('Ready')

            self.print_MessStr("Initialize...")
        
    
    #def onNaviTestBtn(self):
    #    navitest_th = Thread_HollowDisfection(self)
    #    navitest_th.start()
        
    #    navitest_th.navi_runstat.connect(self.hdi_naviRunState)
    #    navitest_th.dsrtraj_runstat.connect(self.hdi_trajRunState)
        
    #    self.print_MessStr("Run navi test thread.")
        

    def CallbackRunNaviCtrlStatus(self, request):
        self.naviStatus = request.status
        mstr = request.status_description
        self.mrstat_lineEdit.setText(mstr)


    def onSaveMapBtn(self):
        os.system("python3 ~/bin/target_term -run 10 rosrun map_server map_saver -f dgcity_sp")
           
        self.print_MessStr("Save map data...")

    def keyReleaseEvent(self, event):
        global convkey
        global convkey_val
        global prev_convkey

        if convkey_val == event.key() and not event.isAutoRepeat():
            prev_convkey = convkey
            convkey = 'r'

            print("release key:" + str(event.key()) + " convkey:" + str(convkey) + " prev_convkey :" + str(prev_convkey))

    
    def keyPressEvent(self, event):
        global bRunMCtrl
        global convkey
        global convkey_val

        if(bRunMCtrl == True):

            key = event.key()
            convkey_val = key
            
            if TeleOP_Cont == True:
            
                if key == Qt.Key_W :
                    convkey = 'w'                
                elif key == Qt.Key_X :
                    convkey = 'x'                
                elif key == Qt.Key_A :
                    convkey = 'a'                
                elif key == Qt.Key_D :
                    convkey = 'd'
                elif key == Qt.Key_U :
                    convkey = 'u'
                elif key == Qt.Key_M :
                    convkey = 'm'
                elif key == Qt.Key_I :
                    convkey = 'i'
                elif key == Qt.Key_O :
                    convkey = 'o'                    
                elif key == Qt.Key_Comma :
                    convkey = ','
                #elif key == Qt.Key_Less :
                #    convkey = ','                
                elif key == Qt.Key_Period :
                    convkey = '.'
                elif key == Qt.Key_Space or key == Qt.Key_S :
                    convkey = 's'
                else:
                    convkey = 0

            else:

                if key == Qt.Key_W :
                    self.target_linear_vel = self.target_linear_vel + self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                elif key == Qt.Key_X :
                    self.target_linear_vel = self.target_linear_vel - self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                elif key == Qt.Key_A :
                    self.target_angular_vel = self.target_angular_vel + self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                elif key == Qt.Key_D :
                    self.target_angular_vel = self.target_angular_vel - self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                elif key == Qt.Key_Space or key == Qt.Key_S :
                    self.target_linear_vel   = 0
                    self.control_linear_vel  = 0
                    self.target_angular_vel  = 0
                    self.control_angular_vel = 0
                    mstr = self.vels(0, 0)
                    self.print_MessStr(mstr)

                if self.target_linear_vel > self.control_linear_vel:
                    self.control_linear_vel = min( self.target_linear_vel, self.control_linear_vel + (0.1/4.0) )
                else:
                    self.control_linear_vel = self.target_linear_vel

                if self.target_angular_vel > self.control_angular_vel:
                    self.control_angular_vel = min( self.target_angular_vel, self.control_angular_vel + (0.1/4.0) )
                else:
                    self.control_angular_vel = self.target_angular_vel

                twist = Twist()
                twist.linear.x = self.control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_angular_vel
                self.pub_twist.publish(twist)


    def onMotionCtrlBtn(self):
        global bRunMCtrl
        global mctrl_th

        if self.bmotionCtrl == False:
            self.bmotionCtrl= True
            bRunMCtrl = True

            mctrl_th = Thread_TeleOP(self)
            mctrl_th.print_lspeed.connect(self.printLinearSpeed)
            mctrl_th.print_aspeed.connect(self.printAngleSpeed)
            mctrl_th.print_lsinc.connect(self.printLSInc)
            mctrl_th.print_asinc.connect(self.printASInc)

            mctrl_th.start()
            
            self.motionctrl_PB.setText('Stop MCtrl')

            self.mcinit_PB.setEnabled(True)

            self.print_MessStr("Run motion control thread.")
            
        else:
            self.bmotionCtrl= False
            bRunMCtrl = False
            self.motionctrl_PB.setText('Motion Ctrl')
            self.mcinit_PB.setDisabled(True)
            self.print_MessStr("Stop motion control thread.")


    def onParkingChargingStationBtn(self):
        if self.bCSParking == False:
            self.bCSParking = True
            
            self.client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            self.client.wait_for_server()

            self.setLaserAct(False)

            goal = ChargingGoal()
            self.client.send_goal(goal)

            self.print_MessStr("Send goal for parking charging station.")
            self.csparking_PB.setText('PCS Cancel')

        else:
            self.bCSParking = False
            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)

            print(cmdstr)

            self.print_MessStr("Cancel parking charge station.")
            self.csparking_PB.setText('Parking C.S.')

            sleep(5)

            twist = Twist()
            twist.linear.x = 0.04; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(16)

            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            self.setLaserAct(True)


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def CallbackRunTrajStatus(self, request):

        print(request)

        stat_str = "none"
        
        if request.curstatus == ZBDSRTrajStatus.IDLING:
            stat_str = "Idling"
        elif request.curstatus == ZBDSRTrajStatus.RUNNING:
            stat_str = "Running"
        elif request.curstatus == ZBDSRTrajStatus.PAUSED:
            stat_str = "Paused"
        elif request.curstatus == ZBDSRTrajStatus.COMPLETED:
            stat_str = "Completed"
        elif request.curstatus == ZBDSRTrajStatus.CANCELLED:
            stat_str = "Canceled"
        elif request.curstatus == ZBDSRTrajStatus.START:
            stat_str = "Start"
        elif request.curstatus == ZBDSRTrajStatus.STOP:
            stat_str = "Stop"
        elif request.curstatus == ZBDSRTrajStatus.ESTOP:
            stat_str = "EStop"
        elif request.curstatus == ZBDSRTrajStatus.RESTART:
            stat_str = "Restart"

        mstr = "TrajNum(" + str(request.trajnum) + "), Status:" + stat_str

        print(mstr)
        
        self.dsrstat_lineEdit.setText(mstr)

    def onZeroPosDSRRobotBtn(self):
        zbdsr_traj = ZBDSRSetTraj()
        zbdsr_traj.trajnum = 101
        zbdsr_traj.setstatus = ZBDSRSetTraj.START
        self.dsrpub.publish(zbdsr_traj)

        mstr = "Zero position of the DSR Robot."
        self.print_MessStr(mstr)

    def onInitPosDSRRobotBtn(self):
        zbdsr_traj = ZBDSRSetTraj()
        zbdsr_traj.trajnum = 100
        zbdsr_traj.setstatus = ZBDSRSetTraj.START
        self.dsrpub.publish(zbdsr_traj)

        mstr = "Initialize position of the DSR Robot."
        self.print_MessStr(mstr)
        
    def print_MessStr(self, str):
        self.messLV.appendRow(QStandardItem(str))
        self.listView.setModel(self.messLV)
        self.listView.scrollToBottom()

    def onWPStartBtn(self):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = self.wpCBStr
        self.ncpub.publish(nc)
        mstr = "[WP] Start way point : " + self.wpCBStr
        self.print_MessStr(mstr)

    def onWPStopBtn(self):
        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.wpCBStr
        self.ncpub.publish(nc)
        mstr = "[WP] Stop way point : " + self.wpCBStr
        self.print_MessStr(mstr)

    # def onWPPauseBtn(self):
    #     nc = NavigationControl()
    #     nc.control = NavigationControl.PAUSE
    #     nc.goal_name = self.wpCBStr
    #     self.ncpub.publish(nc)
    #     mstr = "Pause way point : " + self.wpCBStr
    #     self.print_MessStr(mstr)

    def onDSRTrajRunBtn(self):

        zbdsr_traj = ZBDSRSetTraj()
        val = re.findall(r'\d+', self.dsrtrajCBStr)
        val2 = map(int, val)

        self.cur_trajnum = val2[0]
        zbdsr_traj.trajnum = self.cur_trajnum
        zbdsr_traj.setstatus = ZBDSRSetTraj.START

        self.dsrpub.publish(zbdsr_traj)

        mstr = "Start DSR Robot Trajectory Num : " + str(zbdsr_traj.trajnum)
        self.print_MessStr(mstr)

        if self.bStopStart == True:
            self.bStopStart = False
            self.dsrtraj_stopstartPB.setText('Stop')

    def onDSRTrajStopStartBtn(self):
        if self.bStopStart == False:
            self.bStopStart = True

            zbdsr_traj = ZBDSRSetTraj()            
            zbdsr_traj.trajnum = self.cur_trajnum
            zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
            self.dsrpub.publish(zbdsr_traj)

            self.dsrtraj_stopstartPB.setText('Restart')
            mstr = "Stop DSR Robot Trajectory Num : " + str(zbdsr_traj.trajnum)
            self.print_MessStr(mstr)

        else:
            self.bStopStart = False

            zbdsr_traj = ZBDSRSetTraj()            
            zbdsr_traj.trajnum = self.cur_trajnum
            zbdsr_traj.setstatus = ZBDSRSetTraj.RESTART
            self.dsrpub.publish(zbdsr_traj)

            self.dsrtraj_stopstartPB.setText('Stop')
            mstr = "Restart DSR Robot Trajectory Num : " + str(zbdsr_traj.trajnum)
            self.print_MessStr(mstr)


    def onTrajStartBtn(self):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = self.trajCBStr
        self.ncpub.publish(nc)
        mstr = "Start trajector : " + self.trajCBStr
        self.print_MessStr(mstr)
        
    def onTrajStopBtn(self):
        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.trajCBStr
        self.ncpub.publish(nc)
        mstr = "Stop trajector : " + self.trajCBStr
        self.print_MessStr(mstr)

    # def onTrajPauseBtn(self):
    #     nc = NavigationControl()
    #     nc.control = NavigationControl.PAUSE
    #     nc.goal_name = self.trajCBStr
    #     self.ncpub.publish(nc)
    #     mstr = "Pause trajector : " + self.trajCBStr
    #     self.print_MessStr(mstr)

    def onActivatedwpCB(self, text):
        self.wpCBStr = text

    def onActivatedtrajCB(self, text):
        self.trajCBStr = text

    def onActivateddsrtrajCB(self, text):
        self.dsrtrajCBStr = text

    def onROSCoreBtn(self):
        os.system("gnome-terminal -- roscore")
        self.print_MessStr("Run roscore")

    def onBringupBtn(self):
        if self.bBringUpRun is False:  
            self.bBringUpRun = True      
            os.system("python3 ~/bin/target_term -run 1 roslaunch zetabank_bringup zetabank_robot.launch")
            #os.system("gnome-terminal -- roslaunch zetabank_bringup zetabank_robot.launch")
            self.print_MessStr("Run roslaunch zetabank_robot.launch")
            self.run_bringupPB.setText('Stop Bringup')

            sleep(5)

            self.onUVCBottomOffBtn()

        else:
            self.bBringUpRun = False      
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /STM")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /powerctrl")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zeta_mdrobot_motor_control_node")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /robot_state_publisher")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /sick_safetyscanners_front")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /sick_safetyscanners_rear")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /stm_starter")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zetabank_diagnostics")            
            sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /charging_act")

            self.print_MessStr("Exit zetabank_robot.launch")
            self.run_bringupPB.setText('Run Bringup')
        
    def onSlamBtn(self):
        if self.bSLAMRun is False:
            self.bSLAMRun = True
            os.system("python3 ~/bin/target_term -run 2 roslaunch zetabank_slam zetabank_slam.launch")
            self.print_MessStr("Run roslaunch zetabank_slam.launch")
            self.run_slamPB.setText('Stop SLAM')
        else:
            self.bSLAMRun = False      
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zetabank_slam_gmapping")                        
            self.print_MessStr("Exit roslaunch zetabank_slam.launch")
            self.run_slamPB.setText('Run SLAM')

    def onTeleopBtn(self):
        if self.bTeleOPRun is False:
            self.bTeleOPRun = True
            os.system("python3 ~/bin/target_term -run 3 roslaunch teleop_keyandjoy zetabank_teleop_key.launch")
            self.print_MessStr("Run roslaunch zetabank_teleop_key.launch")
            self.run_teleopPB.setText('Stop teleop')
        else:
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /teleop_key")                        
            self.print_MessStr("Exit roslaunch zetabank_teleop_key.launch")
            self.run_teleopPB.setText('Run teleop')

    def on2LFRBtn(self):
        if self.b2LRFRun is False:
            self.b2LRFRun = True
            os.system("python3 ~/bin/target_term -run 4 roslaunch ira_laser_tools laserscan_multi_merger.launch")
            self.print_MessStr("Run roslaunch laserscan_multi_merger.launch")
            self.run_2LRFPB.setText('Stop 2LRF')
        else:
            self.b2LRFRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /laserscan_multi_merger")             
            self.print_MessStr("Exit roslaunch laserscan_multi_merger.launch")
            self.run_2LRFPB.setText('Act 2LRF')

    def onNavigationBtn(self):
        if self.bNaviRun is False:
            self.bNaviRun = True
            os.system("python3 ~/bin/target_term -run 5 source ~/catkin_ws/devel/setup.bash")

            sleep(2)

            os.system("python3 ~/bin/target_term -run 5 roslaunch zetabank_navigation normal_navigation.launch")
            self.print_MessStr("Run roslaunch normal_navigation.launch")
            self.run_naviPB.setText('Stop Navi.')
        else:
            self.bNaviRun = False            
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /amcl")             
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /map_server")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /move_base")            
            self.print_MessStr("Exit roslaunch normal_navigation.launch")
            self.run_naviPB.setText('Run Navi.')

    def onRVIZBtn(self):
        if self.bRVIZRun is False:
            self.bRVIZRun = True
            
            cmdstr = "python3 ~/bin/target_term -run 6 rosrun rviz rviz -d " + navirviz_fname
            os.system(cmdstr)
            
            self.print_MessStr("Run rosrun rviz")
            self.run_rvizPB.setText('Stop RVIZ')
        else:
            self.bRVIZRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill $(rosnode list | grep rviz_*)")
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill 'rosnode list | grep rviz_*'")
            self.print_MessStr("Stop rosrun rviz")
            self.run_rvizPB.setText('Run RVIZ')

    def onWPCBtn(self):
        if self.bWPCRun is False:
            self.bWPCRun = True
            os.system("python3 ~/bin/target_term -run 7 roslaunch navi_waypoint navigationWayPoint.launch")
            self.print_MessStr("Run roslaunch navigationWayPoint.launch")
            self.run_wpcPB.setText('Stop WPCtrl')            
        else:
            self.bWPCRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /navigation_waypoints_node")                         
            self.print_MessStr("Stop roslaunch navigationWayPoint.launch")
            self.run_wpcPB.setText('Run WPCtrl')

    def onMakeWPBtn(self):
        if self.bMakeWPRun is False:
            self.bMakeWPRun = True
            os.system("python3 ~/bin/target_term -run 8 roslaunch make_waypoint makewaypoint.launch")
            self.print_MessStr("Run roslaunch navigationWayPoint.launch")
            self.run_makeWPPB.setText('Stop makeWP')
        else:
            self.bMakeWPRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /make_waypoint_node")                         
            self.print_MessStr("Stop roslaunch navigationWayPoint.launch")
            self.run_makeWPPB.setText('Run makeWP')

    def onSaveWPBtn(self):
        os.system("python3 ~/bin/target_term -run 10 rostopic pub save_wp std_msgs/String \"save\" -1")
        self.print_MessStr("Run rostopic pub save_wp")

    def onDSRRobotBtn(self):
        if self.bDSRRobotRun is False:
            self.bDSRRobotRun = True
            os.system("python3 ~/bin/target_term -run 9 roslaunch zetabank_dsr zetabank_dsr.launch mode:=real host:=192.168.11.110 port:=12345 model:=m0609 color:=white")
            self.run_dsrRobotPB.setText('Stop DSRRB')
            self.print_MessStr("Run roslaunch zetabank_dsr.launch")
        else:
            self.bDSRRobotRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01/controller_spawner")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01/robot_state_publisher")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01/spawn_create_model")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01/world_tf")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01m0609")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01m0609/controller_spawner")

            os.system("python3 ~/bin/target_term -run 9 kill -2 $$")

            self.run_dsrRobotPB.setText('Run DSR RB')            
            self.print_MessStr("Stop roslaunch zetabank_dsr.launch")

    def onTraj1Btn(self):
        if self.bTraj1Run is False:
            self.bTraj1Run = True
            os.system("python3 ~/bin/target_term -run 11 roslaunch zetabank_dsr zetabank_dsr_control.launch")
            self.print_MessStr("Run roslaunch zetabank_dsr_control.launch")
            self.run_Traj1PB.setText('Stop Traj1')

        else:
            self.bTraj1Run = False

            zbdsr_traj = ZBDSRSetTraj()
            zbdsr_traj.trajnum = self.cur_trajnum
            zbdsr_traj.setstatus = ZBDSRSetTraj.EXIT
            self.dsrpub.publish(zbdsr_traj)

            sleep(0.5)

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr_control")

            os.system("python3 ~/bin/target_term -run 11 kill -2 $$")

            self.print_MessStr("Stop roslaunch zetabank_dsr_control.launch")
            self.run_Traj1PB.setText('Run Traj1')

    def onWarningFieldDisablePBtn(self):
        os.system("gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: 3\" ")
        self.print_MessStr("Disable detecting warning field.")

    def onWarningFieldEnablePBtn(self):
        os.system("gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: 0\" ")
        self.print_MessStr("Enable detecting warning field.")

    def onFrontWFCtrlPBtn(self):
        if self.bFrontWF is False:
            self.bFrontWF = True
            self.SelectWF = ((self.SelectWF & 0b11) | 0b01)
            print(self.SelectWF)

            self.frontwfcPB.setText("Enable Front WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Activate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        else:
            self.bFrontWF = False
            self.SelectWF = ((self.SelectWF & 0b11) & 0b10)
            print(self.SelectWF)

            self.frontwfcPB.setText("Disable Front WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Deactivate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onRearWFCtrlPBtn(self):
        if self.bRearWF is False:
            self.bRearWF = True
            self.SelectWF = ((self.SelectWF & 0b11) | 0b10)
            print(self.SelectWF)

            self.rearwfcPB.setText("Enable Rear WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Activate warning field of rear lidar.")
            self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        else:
            self.bRearWF = False
            self.SelectWF = ((self.SelectWF & 0b11) & 0b01)
            print(self.SelectWF)

            self.rearwfcPB.setText("Disable Rear WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Deactivate warning field of rear lidar.")
            self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onFrontSelWFRangePBtn(self):
        if self.bFrontWF is True:
            if self.bFrontSWF is False:
                self.bFrontSWF = True
                self.SelWFRange = ((self.SelWFRange & 0b11) & 0b10)
                print(self.SelWFRange)

                self.frontselwfrnagePB.setText("Front Range #1")

                cmdstr = "gnome-terminal -- rostopic pub /WarningFieldSelect std_msgs/UInt8 \"data: " + str(self.SelWFRange) + "\" -1 "
                print(cmdstr)
                os.system(cmdstr)
                self.print_MessStr("Select front #1 warning field.")
                self.fran_lineEdit.setText("1")
            else:
                self.bFrontSWF = False
                self.SelWFRange = ((self.SelWFRange & 0b11) | 0b01)
                print(self.SelWFRange)

                self.frontselwfrnagePB.setText("Front Range #0")

                cmdstr = "gnome-terminal -- rostopic pub /WarningFieldSelect std_msgs/UInt8 \"data: " + str(self.SelWFRange) + "\" -1 "
                print(cmdstr)
                os.system(cmdstr)
                self.print_MessStr("Select front #0 warning field.")
                self.fran_lineEdit.setText("0")

    def onRearSelWFRangePBtn(self):
        if self.bRearWF is True:
            if self.bRearSWF is False:
                self.bRearSWF = True
                self.SelWFRange = ((self.SelWFRange & 0b11) & 0b01)
                print(self.SelWFRange)

                self.rearselwfrnagePB.setText("Rear Range #1")

                cmdstr = "gnome-terminal -- rostopic pub /WarningFieldSelect std_msgs/UInt8 \"data: " + str(self.SelWFRange) + "\" -1 "
                print(cmdstr)
                os.system(cmdstr)
                self.print_MessStr("Select rear #1 warning field.")
                self.rran_lineEdit.setText("1")
            else:
                self.bRearSWF = False
                self.SelWFRange = ((self.SelWFRange & 0b11) | 0b10)
                print(self.SelWFRange)

                self.rearselwfrnagePB.setText("Rear Range #0")

                cmdstr = "gnome-terminal -- rostopic pub /WarningFieldSelect std_msgs/UInt8 \"data: " + str(self.SelWFRange) + "\" -1 "
                print(cmdstr)
                os.system(cmdstr)
                self.print_MessStr("Select rear #0 warning field.")
                self.rran_lineEdit.setText("0")


    def onPumpOnPBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 5 'true' -1 "
        # cmdstr = "gnome-terminal -- rostopic pub /pump std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Activate the pump.")
        self.pumpled_label.setPixmap(QtGui.QPixmap(ICON_BLUE_LED))
   
    def onPumpOffPBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 5 'false' -1 "
        # cmdstr = "gnome-terminal -- rostopic pub /pump std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Deactiavte the pump.")
        self.pumpled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onSolOnPBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 14 'true' -1 "
        # cmdstr = "gnome-terminal -- rostopic pub /sol std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Activate the solenoid valve.")
        self.solled_label.setPixmap(QtGui.QPixmap(ICON_ORANGE_LED))

    def onSolOffPBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 14 'false' -1 "
        # cmdstr = "gnome-terminal -- rostopic pub /sol std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Deactiavte the solenoid valve.")
        self.solled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onUVCEndtoolOnBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 15 'true' -1 "
        # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the UVC of end effector.")
        self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))

    def onUVCEndtoolOffBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 15 'false' -1 "
        # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the UVC of end effector.")
        self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onUVCBottomOnBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 0 'true' -1 "
        #cmdstr = "gnome-terminal -- rostopic pub /uvc_bottom std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the UVC of bottom.")
        self.UVCBled_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))

    def onUVCBottomOffBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 0 'false' -1 "
        #cmdstr = "gnome-terminal -- rostopic pub /uvc_bottom std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the UVC of bottom.")
        self.UVCBled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onScrubberCtrlOnBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 6 'true' -1 "
        # cmdstr = "gnome-terminal -- rostopic pub /ScrubberControl std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the Scrubber.")
        self.Scrubled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

    def onScrubberCtrlOffBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 6 'false' -1 "
        # cmdstr = "gnome-terminal -- rostopic pub /ScrubberControl std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn off the Scrubber.")
        self.Scrubled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onCoonectSerial1Btn(self):
        os.system("gnome-terminal -- rosrun rosserial_python serial_node.py _baud:=115200 _port:=\"/dev/ttyACM0\" __name:=\"imu\" ")
        self.print_MessStr("Connect to the safety Lidar serial comm.(ttyACM0)")

    def onCoonectSerial2Btn(self):
        os.system("gnome-terminal -- rosrun rosserial_python serial_node.py _baud:=9600 _port:=\"/dev/ttyUSB-IOCtrl\" __name:=\"ioctrl\" ")
        self.print_MessStr("Connect to the I/O module controller serial comm.(ttyUSB-IOCtrl)")


    def onExit(self):
        self.print_MessStr("Exit program...")
        os.system("python3 ~/bin/target_term -run 1 exit")
        os.system("python3 ~/bin/target_term -run 2 exit") 
        os.system("python3 ~/bin/target_term -run 3 exit") 
        os.system("python3 ~/bin/target_term -run 4 exit") 
        os.system("python3 ~/bin/target_term -run 5 exit")
        os.system("python3 ~/bin/target_term -run 6 exit") 
        os.system("python3 ~/bin/target_term -run 7 exit") 
        os.system("python3 ~/bin/target_term -run 8 exit") 
        os.system("python3 ~/bin/target_term -run 9 exit")
        os.system("python3 ~/bin/target_term -run 10 exit")
        os.system("python3 ~/bin/target_term -run 11 exit")
        os.system("python3 ~/bin/target_term -run 12 exit")

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        self.close()


app = QApplication(sys.argv)
window = MyWindow()
window.show()
sys.exit(app.exec_())
