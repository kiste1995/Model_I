#!/usr/bin/env python

import os, sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt
#from PyQt5.uic.properties import QtGui
from PyQt5 import QtGui
#from PyQt5 import QtCore, QtGui
#from PyQt5.QtCore import QCoreApplication
from python_qt_binding import loadUi
import time
import yaml
import rospy
from zetabank_msgs.msg import NavigationControl, NavigationControlStatus
import os
import numpy as np
import re

#from zetabank_msgs.msg import ZBDSRTraj
#from zetabank_msgs.srv import DSRobot, DSRobotRequest
from zetabank_msgs.msg import ZBDSRSetTraj, ZBDSRTrajStatus
from autocharge.msg import ChargingAction, ChargingActionGoal, ChargingGoal
import actionlib
import threading
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import std_msgs
from std_msgs.msg import Bool, String
#from std_msgs.msg import String


rospy.init_node('naviGUI_node')

# zetabank_robotdir = rospy.get_param('~zetabankrobot_dir')
# rviz_fname = zetabank_robotdir + "/zetabank_navigation/rviz/zetabank_nav.rviz"

rviz_fname = "/home/zetabank/catkin_ws/src/zetabank_robot_control/zetabank_navigation/rviz/zetabank_nav.rviz"

rootdir = rospy.get_param('~root_dir')
#print("root dir : " + rootdir)

#ui_dir = os.path.dirname(os.path.realpath(__file__))
#print ("current dir : " + ui_dir)

fname = rootdir + "/scripts/ncwp_gui.ui"
#fname = ui_dir + "/ncwp_gui.ui"
#print("fname : " + fname)

#yaml_fname = rootdir +"/config/HDIWP5.yaml"
yaml_fname = rootdir +"/config/HDIWP7.yaml"
#print("yaml file name : " + yaml_fname)

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
        'r':(0,0),
        }

speedBindings={
        'u':(1.1,1.1),
        'm':(.9,.9),
        'i':(1.1,1),
        ',':(.9,1),
        'o':(1,1.1),
        '.':(1,.9),
        }

TeleOP_Cont = True        

convkey = 0
prev_convkey = 0
convkey_val = 0
bRunMCtrl = False

DTRunTime = 10

bHDIWPFileLoadOK = False
bCRWPFileLoadOK = False

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
    
    def __init__(self, parent = None):
        super(Thread_ConfRoomDisfection, self).__init__(parent)
        #self.working = True
        
        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)

        rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)

        self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRSetTraj, queue_size = 10)

        rospy.Subscriber("/status_sdr_traj", ZBDSRTrajStatus, self.CallbackRunTrajStatus)

        rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)

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

        
    def StopWaypoint(self):
        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.wpName
        self.ncpub.publish(nc)
        mstr = "[WP] Stop way point : " + self.wpCBStr
        self.navi_runstat.emit(mstr)

    def CallbackCSStatus(self, request):
        self.csStatus = request.data

        mstr = "Charging Station Status : " + self.csStatus
        #print(mstr)

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
        #self.working = False
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
                        
        
    def run(self):
        runcnt = 0

        if self.bRunFlag == True:

            self.naviStatus = NavigationControlStatus.IDLING
            
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
            
            # ========================================
            # Conference Room Disinfection Process #1
            # ========================================

            self.gotoWayPoint("CRWPDoor")

            sleep(5)

            while True:
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
                self.trajStatus = ZBDSRTrajStatus.IDLING
                sleep(2)

                self.RunDSRTraj(5)

                sleep(8)

                print("Moving Trjaectory #5...")
        
              
        # ========================================
        # Conference Room Disinfection Process #3
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
            
            self.gotoWayPoint("DSOLON1")

            sleep(5)
 
            while True:
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
                # self.PumpOn()

                print("Turn on Pump...")
                
                # sleep(1)

                # self.SolOn()
                
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
                # self.SolOff()
                
                print("Turn off Sol. valve...")
                
                # self.PumpOff()

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
        # Conference Room Disinfection Process #6
        # ========================================
        
        if self.bRunFlag == True:
            runcnt = 0

            self.naviStatus = NavigationControlStatus.IDLING
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("DSOLON2")

            sleep(5)
            
            while True:
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
                # self.PumpOn()

                print("Turn on Pump...")
                
                # sleep(1)

                # self.SolOn()
                
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
                # self.SolOff()
                
                print("Turn off Sol. valve...")
                
                # self.PumpOff()

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
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRWP1 CRWP3 process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause CRWP1 CRWP3 process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart CRWP3 CSDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.CRDIRun[3] = 1
                    
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
                        self.CRDIRun[3] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) CRWP3")

                        break

                sleep(0.1)
                
            if self.bRunFlag == True:
                # self.onSolOffPBtn()
                
                print("Turn on Sol. valve...")
                
                # self.pumpOffPB()

                print("Turn on Pump...")

                zbdsr_traj = ZBDSRSetTraj()            
                zbdsr_traj.trajnum = 5
                zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                self.dsrpub.publish(zbdsr_traj)

                sleep(2)

                
                self.trajStatus = ZBDSRTrajStatus.IDLING
                sleep(2)

                self.RunDSRTraj(6)

                # while self.bRunFlag:
                #     if self.bRunFlag == False:
                #         self.StopWaypoint()
                        
                #         sleep(1)
                        
                #         self.stop()
                        
                #     if self.bRunPauseFlag == True:
                #         while self.bRunPauseFlag:
                #             if self.bRunPauseFlag is False:
                #                 break
                        
                #             sleep(0.1)

                while True:

                    if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                        
                        break

                    sleep(0.1)

                print("Complete moving Trjaectory #6...")
                
                sleep(5)
        
        
        # ========================================
        # Conference Room Disinfection Process #9
        # ========================================
        if self.bRunFlag == True:
            self.trajStatus = ZBDSRTrajStatus.IDLING

            sleep(2)

            self.RunDSRTraj(7)

            sleep(8)

            print("Complete moving DSR Trajaectory #7...")
            
            sleep(2)
            
            # self.PumpOn()

            # print("Turn on Pump...")
            
            # sleep(1)

            # self.SolOn()
            
            # print("Turn on Sol. valve...")
            
            # ==========================================
            # Conference Room Disinfection Process #10
            # ==========================================
            
            self.naviStatus = NavigationControlStatus.IDLING
            
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("CRTRAJ1")

            sleep(5)

            runcount = 0

            while True:
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop CRTRAJ1 CSDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    break
            
                #if self.naviStatus == NavigationControlStatus.COMPLETED:
                if self.naviStatus == NavigationControlStatus.TRAJCOMPLETED:

                    #runcount = runcount + 1

                    #print("Run count : {0}".format(runcount))

                    print("navi Status : " + str(self.naviStatus))

                    self.naviStatus = NavigationControlStatus.IDLING

                    # if runcount >= 7 :
                    #     break

                    break

                sleep(0.1)

            if self.bRunFlag == True:
                print("======>>>>> Complete driving trajectory #1...")
                

                # self.SolOff()
                
                # print("Turn on Sol. valve...")
                
                # self.PumpOff()

                # print("Turn on Pump...")


                zbdsr_traj = ZBDSRSetTraj()            
                zbdsr_traj.trajnum = 7
                zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                self.dsrpub.publish(zbdsr_traj)

                sleep(2)

                self.trajStatus = ZBDSRTrajStatus.IDLING
                sleep(2)

                self.RunDSRTraj(8)

                # while self.bRunFlag:
                #     if self.bRunFlag == False:
                #         self.StopWaypoint()
                        
                #         sleep(1)
                        
                #         self.stop()
                        
                #     if self.bRunPauseFlag == True:
                #         while self.bRunPauseFlag:
                #             if self.bRunPauseFlag is False:
                #                 break
                        
                #             sleep(0.1)

                while True:

                    if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                        
                        break

                    sleep(0.1)

                print("Complete moving Trjaectory #8...")
        
        
        # ========================================
        # Conference Room Disinfection Process #11
        # ========================================

        if self.bRunFlag == True:
            self.naviStatus = NavigationControlStatus.IDLING

            self.gotoWayPoint("CRExitPos")

            sleep(5)
            
            while True:
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
        # Conference Room Disinfection Process #12
        # ========================================
        
        if self.bRunFlag == True:
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("chargingpos")

            sleep(5)

            runcnt = 0
            
            while True:
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
        # Conference Room Disinfection Process #13
        # ========================================

        if self.bRunFlag == True:
            #self.onParkingChargingStationBtn()

            client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            client.wait_for_server()

            goal = ChargingGoal()
            client.send_goal(goal)

            print("Parking charging station...")

            while True:
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

    def __init__(self, parent = None):
        super(Thread_HollowDisfection, self).__init__(parent)
        #self.working = True
        
        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)

        rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)

        self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRSetTraj, queue_size = 10)

        rospy.Subscriber("/status_sdr_traj", ZBDSRTrajStatus, self.CallbackRunTrajStatus)

        rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)

        self.naviStatus = NavigationControlStatus.IDLING
        self.prevnaviStatus = NavigationControlStatus.IDLING
        self.trajStatus = ZBDSRSetTraj.IDLING

        self.HDIRun = np.chararray(10)
        self.HDIRun[:] = 0
        
        self.bRunFlag = True
        self.bRunPauseFlag = False

        self.csStatus = "none"

        self.wpName = "none"
        
        self.RepCnt = 5
        

    def CallbackRunNaviCtrlStatus(self, request):
        self.naviStatus = request.status

        if(self.prevnaviStatus != self.naviStatus):
            mstr = request.status_description
            print(mstr)

            self.navi_runstat.emit(mstr)

            self.prevnaviStatus = self.naviStatus
            #self.mrstat_lineEdit.setText(mstr)

    def CallbackCSStatus(self, request):
        self.csStatus = request.data

        mstr = "Charging Station Status : " + self.csStatus
        #print(mstr)

        self.navi_runstat.emit(mstr)


    def CallbackRunTrajStatus(self, request):

        #print(request)

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
        self.navi_runstat.emit(mstr)

        
    def gotoWayPoint(self, wpname):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = wpname
        self.ncpub.publish(nc)
        
        self.wpName = wpname
        
        mstr = "[WP] Start hollow disinfection way point : " + nc.goal_name
        print(mstr)
        #self.print_MessStr(mstr)
        
        #while True:
        
        #    if self.naviStatus == NavigationControlStatus.COMPLETED
        #        break;

    def RunDSRTraj(self, trajnum):
        zbdsr_traj = ZBDSRSetTraj()
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
                        
    def navi_Process(self):
        runcnt = 0

        bErrFlag = False
    
        # =================================
        # Hallow Disinfection Process #1
        # =================================

        self.naviStatus = NavigationControlStatus.IDLING


        #print("HDIRun[0] : " + str(self.HDIRun[0]))

        if self.HDIRun[0] == '0' and self.bRunFlag == True:        
            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device1")

            sleep(5)

            while True:
                if self.bRunFlag == False:
                    self.StopWaypoint()
                    
                    self.navi_runstat.emit("Stop Device1 HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)
                    
                    self.stop()

                    bErrFlag = True

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause Device1 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart Device1 HDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.HDIRun[0] = 1

                    print("Arrived at Device1...")

                    bErrFlag = False
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device1")

                        self.navi_runstat.emit("Restart Device1")

                        print("Restart Device1")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.HDIRun[0] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) Device1")

                        print("Error(WARNNRGTO) Device1")

                        bErrFlag = True

                        break

                # if self.naviStatus == NavigationControlStatus.ERRGTGF:
                #     self.navi_runstat.emit("Error(ERRGTGF) Device1")

                #     self.HDIRun[0] = 0

                #     break

                sleep(0.1)

            if bErrFlag == False and self.bRunFlag == True: 
                #sleep(5)

                sleep(2)

                self.trajStatus = ZBDSRTrajStatus.IDLING
                
                self.RunDSRTraj(1)

                while True:
                    if self.bRunFlag == False:
                        self.stopTraj()

                        self.dsrtraj_runstat.emit("Stop moving HDI traj. #1...")
                            
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

                    self.dsrtraj_runstat.emit("Complete moving HDI Trjaectory #1...")
                    #print("Complete moving Trjaectory #3...")

                    # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: true\" -1 "
                    # print(cmdstr)
                    # os.system(cmdstr)

                    # print("Turn on UV Lamp...")

                    # print("Waiting during disinfection...")

                    # sleep(DTRunTime)

                    # print("Done first device disinfection...")

                    # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: false\" -1 "
                    # print(cmdstr)
                    # os.system(cmdstr)

                    # print("Turn off UV Lamp...")

                    sleep(2)

                    zbdsr_traj = ZBDSRSetTraj()            
                    zbdsr_traj.trajnum = 1
                    zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                    self.dsrpub.publish(zbdsr_traj)

                    sleep(2)

                    self.RunDSRTraj(2)

                    self.trajStatus = ZBDSRTrajStatus.IDLING

                    while True:
                    #while self.bRunFlag:
                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.dsrtraj_runstat.emit("Stop moving HDI traj. #2...")
                            
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
                        print("Complete moving HDI Trjaectory #2...")

                        print("Complete first device disinfection...")

            else:
                print("Skip first device disinfection...")

        bErrFlag = False

        # =================================
        # Hallow Disinfection Process #2
        # =================================

        if self.HDIRun[1] == '0' and self.bRunFlag == True:   
            #self.trajStatus = ZBDSRTrajStatus.IDLING
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device2")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bRunFlag == False:
                    self.StopWaypoint()

                    self.navi_runstat.emit("Stop Device2 HDI process...")

                    self.setrunflag.emit(True)
                    
                    sleep(1)
                    
                    self.stop()

                    bErrFlag = True

                    break                    
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause Device2 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart Device2 HDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.HDIRun[1] = 1

                    print("Arrived at Device2...")

                    bErrFlag = False
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device2")

                        self.navi_runstat.emit("Restart Device2")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.HDIRun[1] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) Device2")

                        bErrFlag = True

                        break

                # if self.naviStatus == NavigationControlStatus.ERRGTGF:
                #     self.navi_runstat.emit("Error(ERRGTGF) Device2")

                #     self.HDIRun[1] = 0

                sleep(0.1)


            if bErrFlag == False and self.bRunFlag == True:
 
                sleep(2)

                self.trajStatus = ZBDSRTrajStatus.IDLING

                self.RunDSRTraj(1)

                while True:
                    if self.bRunFlag == False:
                        self.stopTraj()

                        self.dsrtraj_runstat.emit("Stop moving HDI traj. #1...")
                            
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
                    print("Complete moving HDI Trjaectory #1...")

                    # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: true\" -1 "
                    # print(cmdstr)
                    # os.system(cmdstr)

                    # print("Turn on UV Lamp...")

                    # print("Waiting during disinfection...")

                    # sleep(DTRunTime)

                    # print("Done first device disinfection...")

                    # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: false\" -1 "
                    # print(cmdstr)
                    # os.system(cmdstr)

                    # print("Turn off UV Lamp...")

                    sleep(2)

                    zbdsr_traj = ZBDSRSetTraj()            
                    zbdsr_traj.trajnum = 1
                    zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                    self.dsrpub.publish(zbdsr_traj)

                    sleep(2)

                    self.RunDSRTraj(2)

                    self.trajStatus = ZBDSRTrajStatus.IDLING

                    while True:
                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.dsrtraj_runstat.emit("Stop moving HDI traj. #2...")
                                
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
                        print("Complete moving HDI Trjaectory #2...")        

                        print("Complete second device disinfection...")

            else:
                print("Skip second device disinfection...")

        bErrFlag = False

        # =================================
        # Hallow Disinfection Process #4
        # =================================
        
        if (self.HDIRun[2] == '0' or self.HDIRun[3] == '0') and self.bRunFlag == True:   
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("lobbypos")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bRunFlag == False:
                    self.stopTraj()

                    self.navi_runstat.emit("Stop lobbypos HDI process...")

                    self.setrunflag.emit(True)

                    sleep(1)

                    self.stop()

                    bErrFlag = True

                    break

                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause lobbypos HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart lobbypos HDI process...")
                            break
                    
                        sleep(0.1)            
        
                if self.naviStatus == NavigationControlStatus.COMPLETED:
                    self.HDIRun[2] = 1

                    print("Arrived at lobbypos...")
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("lobbypos")

                        self.navi_runstat.emit("Restart lobbypos")

                    else:
                        self.HDIRun[2] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) lobbypos")

                    break

                if self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.navi_runstat.emit("Error(ERRGTGF) lobbypos")

                    self.HDIRun[2] = 0

                sleep(0.1)


            if self.bRunFlag == True:
                sleep(2)

        # =================================
        # Hallow Disinfection Process #5
        # =================================

        if self.HDIRun[3] == '0' and self.bRunFlag == True: 
            self.trajStatus = ZBDSRTrajStatus.IDLING
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device3")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bRunFlag == False:
                    self.StopWaypoint()

                    self.navi_runstat.emit("Stop Device3 HDI process...")

                    self.setrunflag.emit(True)
                    
                    sleep(1)
                    
                    self.stop()

                    bErrFlag = True

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause Device3 HDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart Device3 HDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED:

                    self.HDIRun[3] = 1

                    print("Arrived at Device3...")

                    bErrFlag = False
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device3")

                        self.navi_runstat.emit("Restart Device3")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.HDIRun[3] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) Device3")

                        bErrFlag = True

                        break

                # if self.naviStatus == NavigationControlStatus.ERRGTGF:
                #     self.navi_runstat.emit("Error(ERRGTGF) Device3")

                #     self.HDIRun[3] = 0

                sleep(0.1)


            if bErrFlag == False and self.bRunFlag == True:
                sleep(5)

                self.RunDSRTraj(3)

                while True:
                    if self.bRunFlag == False:
                        self.stopTraj()
                        
                        self.dsrtraj_runstat.emit("Stop moving HDI traj. #3...")

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
                    print("Complete moving HDI Trjaectory #3...")

                    # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: true\" -1 "
                    # print(cmdstr)
                    # os.system(cmdstr)

                    # print("Turn on UV Lamp...")

                    # print("Waiting during disinfection...")

                    # sleep(DTRunTime)

                    # print("Done first device disinfection...")

                    # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: false\" -1 "
                    # print(cmdstr)
                    # os.system(cmdstr)

                    # print("Turn off UV Lamp...")

                    sleep(2)

                    zbdsr_traj = ZBDSRSetTraj()            
                    zbdsr_traj.trajnum = 3
                    zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                    self.dsrpub.publish(zbdsr_traj)

                    sleep(2)

                    self.RunDSRTraj(4)

                    self.trajStatus = ZBDSRTrajStatus.IDLING

                    while True:
                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.dsrtraj_runstat.emit("Stop moving HDI traj. #4...")
                            
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
                        print("Complete moving HDI Trjaectory #4...")        

                        print("Complete third device disinfection...")

            else:
                print("Skip third device disinfection...")

        bErrFlag = False

        # =================================
        # Hallow Disinfection Process #6
        # =================================

        if self.HDIRun[4] == '0' and self.bRunFlag == True: 
            self.trajStatus = ZBDSRTrajStatus.IDLING
            self.naviStatus = NavigationControlStatus.IDLING

            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

            self.gotoWayPoint("device4")

            sleep(5)

            runcnt = 0
            
            while True:
                if self.bRunFlag == False:
                    self.StopWaypoint()

                    self.navi_runstat.emit("Stop Device4 HDI process...")

                    self.setrunflag.emit(True)
                    
                    sleep(1)
                    
                    self.stop()

                    bErrFlag = True

                    break
                    
                if self.bRunPauseFlag == True:
                    self.navi_runstat.emit("Pause Device4 hDI process...")
                    while self.bRunPauseFlag:
                        if self.bRunPauseFlag is False:
                            self.navi_runstat.emit("Restart Device4 HDI process...")
                            break
                    
                        sleep(0.1)
            
                if self.naviStatus == NavigationControlStatus.COMPLETED or self.naviStatus == NavigationControlStatus.ERRGTGF:
                    self.HDIRun[4] = 1

                    print("Arrived at Device5...")
                    bErrFlag = False
                    break

                if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                    runcnt = runcnt + 1

                    if runcnt <= self.RepCnt:
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                        self.gotoWayPoint("device4")

                        self.navi_runstat.emit("Restart Device4")

                        self.naviStatus = NavigationControlStatus.IDLING

                    else:
                        self.HDIRun[4] = 0

                        runcnt = 0

                        self.navi_runstat.emit("Error(WARNNRGTO) Device4")

                        bErrFlag = True

                        break

                # if self.naviStatus == NavigationControlStatus.ERRGTGF:
                #     self.navi_runstat.emit("Error(ERRGTGF) Device4")

                #     self.HDIRun[4] = 0

                sleep(0.1)

            if bErrFlag == False and self.bRunFlag == True:
                sleep(5)

                self.RunDSRTraj(3)

                while True:
                    if self.bRunFlag == False:
                        self.stopTraj()

                        self.dsrtraj_runstat.emit("Stop moving HDI traj. #3...")
                        
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

                    print("Complete moving HDI Trjaectory #3...")

                    # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: true\" -1 "
                    # print(cmdstr)
                    # os.system(cmdstr)

                    # print("Turn on UV Lamp...")

                    # print("Waiting during disinfection...")

                    # sleep(DTRunTime)

                    # print("Done first device disinfection...")

                    # cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: false\" -1 "
                    # print(cmdstr)
                    # os.system(cmdstr)

                    # print("Turn off UV Lamp...")

                    sleep(2)

                    zbdsr_traj = ZBDSRSetTraj()            
                    zbdsr_traj.trajnum = 3
                    zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                    self.dsrpub.publish(zbdsr_traj)

                    sleep(2)

                    self.RunDSRTraj(4)

                    self.trajStatus = ZBDSRTrajStatus.IDLING

                    while True:
                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.dsrtraj_runstat.emit("Stop moving HDI traj. #4...")
                            
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
                        print("Complete moving HDI Trjaectory #4...")   

            else:
                print("Skip moving HDI Trjaectory #4...")   

       

    def gotoChargingStation(self):
        # =================================
        # Hallow Disinfection Process #7
        # =================================
        
        self.naviStatus = NavigationControlStatus.IDLING

        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #self.gotoWayPoint("startpose")
        self.gotoWayPoint("chargingpos")

        sleep(5)

        runcnt = 0
        
        while True:        
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

                    #self.gotoWayPoint("startpose")
                    self.gotoWayPoint("chargingpos")

                    self.navi_runstat.emit("Restart chargingpos")

                else:
                #    self.HDIRun[6] = 0

                    runcnt = 0

                    self.navi_runstat.emit("Error(WARNNRGTO) chargingpos")

                break

            # if self.naviStatus == NavigationControlStatus.ERRGTGF:
            #     self.navi_runstat.emit("Error(ERRGTGF) chargingpos")

            #     self.HDIRun[6] = 0

            sleep(0.1)

        if self.bRunFlag == True:

            print("Arrived at charging station position...")  

            client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            client.wait_for_server()

            goal = ChargingGoal()
            client.send_goal(goal)

            print("Parking charging station...")

            while True:
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

                    self.setapflag.emit(True)
                    self.setrunflag.emit(False)                   
                    
                    break
                    
                sleep(0.1)
    

    def run(self):

        for i in range(1, 10):
            self.HDIRun[i] = 0

        self.navi_Process()

        if self.bRunFlag == True:

            print("First time : Complete forth device disinfection...")
                
            if self.HDIRun[0] == '0' or self.HDIRun[1] == '0' or self.HDIRun[2] == '0' or self.HDIRun[3] == '0' or self.HDIRun[4] == '0' or self.HDIRun[5] == '0':

                self.navi_Process()

            print("Second time : Complete forth device disinfection...")
        
        
            print("Complete forth device disinfection...")

            print("Go to charging station....")
        
            self.gotoChargingStation()

            print("Complete all disinfection processes in the hallow disindection.")
  

    
    def stop(self):
        #self.working = False
        self.quit()
        self.wait(1000)
        



class Thread_TeleOP(QThread):
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
        self.speed = 0.1
        self.turn = 0.15
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

    def run(self):
        global convkey
        global prev_convkey
        global bRunMCtrl

        while bRunMCtrl == True:
            if convkey in moveBindings.keys():
                self.x = moveBindings[convkey][0]
                self.th = moveBindings[convkey][1]

                #self.count = 0
                print vels(self.control_speed, self.control_turn)
                # prev_convkey = convkey

            elif convkey in speedBindings.keys():
                self.speed = self.speed * speedBindings[convkey][0]              
                self.turn = self.turn * speedBindings[convkey][1]
                #self.count = 0
                
                print vels(self.speed, self.turn)
                # prev_convkey = convkey

            elif convkey == 's' :
                self.x = 0
                self.th = 0
                self.control_speed = 0
                self.control_turn = 0

            elif convkey == 'r':
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

            twist = Twist()
            twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
            self.pub_twist.publish(twist)

        print("Exit thread of TeleOP...")

        
#class MyWindow(QtGui.QWidget):
class MyWindow(QMainWindow, form_class):
    def __init__(self):
        super(QMainWindow, self).__init__()
        self.setupUi(self)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        #ui_dir = os.path.dirname(os.path.realpath(__file__))
        #print ui_dir

        self.settings = termios.tcgetattr(sys.stdin)

        self.bFrontWF = False
        self.bRearWF = False
        self.bFrontSWF = False
        self.bRearSWF = False
        self.SelectWF = 0b00
        self.SelWFRange = 0b11

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

        # self.convkey = 0

        # print("read map file")

        # print("yaml file name : " + yaml_fname)

        try :
            with open(yaml_fname) as f:
                wp_list = yaml.safe_load(f)
                #wp_list = yaml.load(f, Loader=yaml.FullLoader)
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

            for i in range(1, 9):
                trajname =  "Trajectory {0}".format(i)
                #print(trajname)
                self.dsrtraj_CB.addItem(trajname)
        except :
            bHDIWPFileLoadOK = False
            
        finally:
            bHDIWPFileLoadOK = True
        


        self.messLV = QStandardItemModel()

        #rospy.init_node('ncWPGUI')
        #rospy.on_shutdown(shutdonw)

        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)

        #self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRTraj, queue_size = 10)
        self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRSetTraj, queue_size = 10)

        rospy.Subscriber("/status_sdr_traj", ZBDSRTrajStatus, self.CallbackRunTrajStatus)

        rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)
        
        # self.dsrservice_client = rospy.ServiceProxy("/control_dsr_traj", DSRobot)
        # self.dsrrequest_srv = DSRobotRequest()

        self.wpCBStr = self.wp_CB.currentText()
        self.trajCBStr = self.traj_CB.currentText()
        self.dsrtrajCBStr = self.dsrtraj_CB.currentText()

        self.wp_CB.activated[str].connect(self.onActivatedwpCB)
        self.traj_CB.activated[str].connect(self.onActivatedtrajCB)
        self.dsrtraj_CB.activated[str].connect(self.onActivateddsrtrajCB)

        self.print_MessStr("Compelte initialization...")

        self.wp_startPB.clicked.connect(self.onWPStartBtn)
        self.wp_stopPB.clicked.connect(self.onWPStopBtn)
        self.wp_pausePB.clicked.connect(self.onWPPauseBtn)

        self.dsrtraj_runPB.clicked.connect(self.onDSRTrajRunBtn)
        self.dsrtraj_stopstartPB.clicked.connect(self.onDSRTrajStopStartBtn)

        
        self.traj_startPB.clicked.connect(self.onTrajStartBtn)
        self.traj_stopPB.clicked.connect(self.onTrajStopBtn)
        self.traj_pausePB.clicked.connect(self.onTrajPauseBtn)

        self.run_roscorePB.clicked.connect(self.onROSCoreBtn)
        self.run_bringupPB.clicked.connect(self.onBringupBtn)
        self.run_slamPB.clicked.connect(self.onSlamBtn)
        self.run_teleopPB.clicked.connect(self.onTeleopBtn)
        self.run_2LRFPB.clicked.connect(self.on2LFRBtn)
        self.run_naviPB.clicked.connect(self.onNavigationBtn)
        self.run_rvizPB.clicked.connect(self.onRVIZBtn)
        self.run_wpcPB.clicked.connect(self.onWPCBtn)
        self.run_makeWPPB.clicked.connect(self.onMakeWPBtn)

        #self.wfcDisablePB.clicked.connect(self.onWarningFieldDisablePBtn)
        #self.wfcEnablePB.clicked.connect(self.onWarningFieldEnablePBtn)

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
        

        
        # run 8 terminal window
        os.system("python3 ~/bin/target_term -set 12")
        self.print_MessStr("Run terminal windows.")

        data = os.environ["HOME"]+"/.term_list"
        self.t_term = open(data).read().splitlines()
        print(self.t_term)

        self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        
        #self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        #self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        
        self.pumpled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        #self.pumpled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        self.solled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        #self.solled_label.setPixmap(QtGui.QPixmap(ICON_ORANGE_LED))
        self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.UVCBled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        #self.UVCled_label.setPixmap(QtGui.QPixmap(ICON_YELLOW_LED))
        self.Scrubled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

        self.fran_lineEdit.setText("0")
        self.rran_lineEdit.setText("0")

        self.naviHDI_th = Thread_HollowDisfection(self)
        self.naviCRDI_th = Thread_ConfRoomDisfection(self)

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
        self.bCSDIRun = val
        self.print_MessStr("Stop CSDI process...")

    @pyqtSlot(bool)
    def setHDIAutoParkingFlag(self, val):
        self.bHDIAutoPark = val
        print("Result of HDI Auto Parking : " + str(val))

    @pyqtSlot(bool)
    def setCSDIAutoParkingFlag(self, val):
        self.bCSDIAutoPark = val
        print("Result of CSDI Auto Parking : " + str(val))


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

        # else:
        #     self.bStopFlag = False


    def onAutoParkingBtn(self):
        autopark_th = Thread_ReturnToCharingStation(self)

        if self.bAutoPark == False:
            self.bAutoPark = True
            autopark_th.setRunType(1)
            self.print_MessStr("Send auto parking command.")
            self.csparking_PB.setText('Cancel')
        else:
            self.bAutoPark = False
            autopark_th.setRunType(0)
            self.print_MessStr("Send cancel parking command.")
            self.csparking_PB.setText('Auto C.S.')

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
        
            self.naviCRDI_th = Thread_ConfRoomDisfection(self)
            self.naviCRDI_th.start()
            
            self.naviCRDI_th.navi_runstat.connect(self.crdi_naviRunState)
            self.naviCRDI_th.dsrtraj_runstat.connect(self.crdi_trajRunState)
            self.naviCRDI_th.setrunflag.connect(self.setCSDIRunFlag)
            self.naviCRDI_th.setapflag.connect(self.setCSDIAutoParkingFlag)

            # naviCRDI_th = Thread_ConfRoomDisfection(self)
            # naviCRDI_th.start()
            
            # naviCRDI_th.navi_runstat.connect(self.crdi_naviRunState)
            # naviCRDI_th.dsrtraj_runstat.connect(self.crdi_trajRunState)
            
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
            
            self.naviHDI_th.start()
            self.naviHDI_th.navi_runstat.connect(self.hdi_naviRunState)
            self.naviHDI_th.dsrtraj_runstat.connect(self.hdi_trajRunState)
            self.naviHDI_th.setrunflag.connect(self.setHDIRunFlag)
            self.naviHDI_th.setapflag.connect(self.setHDIAutoParkingFlag)
            
            # naviHDI_th = Thread_HollowDisfection(self)
            # naviHDI_th.start()
            
            # naviHDI_th.navi_runstat.connect(self.hdi_naviRunState)
            # naviHDI_th.dsrtraj_runstat.connect(self.hdi_trajRunState)
            # naviHDI_th.setrunflag.connect(self.setHDIRunFlag)
            
            self.print_MessStr("Run navi Holley Disinfection Process thread.")


    def onReadyBtn(self):
        if self.bReadyRun == False:
            self.bReadyRun = True

            self.onDSRRobotBtn()
            sleep(5)

            self.onTraj1Btn()
            sleep(3)

            self.onInitPosDSRRobotBtn()
            sleep(2)

            self.onBringupBtn()
            sleep(4)
            
            self.on2LFRBtn()
            sleep(2)
            
            self.onNavigationBtn()
            sleep(3)
            
            self.onRVIZBtn()
            sleep(5)
            
            self.onWPCBtn()
            sleep(4)

           
            self.readyPB.setText('Init')

            self.print_MessStr("Ready to initialize...")

        else:
            self.bReadyRun = False

            self.onTraj1Btn()

            self.onDSRRobotBtn()

            self.onWPCBtn()

            self.onRVIZBtn()

            self.onNavigationBtn()

            self.on2LFRBtn()

            self.onBringupBtn()

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


    
    #def MotionControl_callback(self):

    def keyReleaseEvent(self, event):
        global convkey
        global convkey_val

        #print("release key : " + str(event.key()))
        #print("conv key : " + str(convkey))

        if convkey_val == event.key() and not event.isAutoRepeat():
            convkey = 'r'

            print("release key:" + str(event.key()) + " convkey:" + str(convkey))

    
    def keyPressEvent(self, event):
        global bRunMCtrl
        global convkey
        global convkey_val

        #print(event.text())
    
        #while(self.bRunMCtrl):
        if(bRunMCtrl == True):
            #key = self.getKey()

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
                elif key == Qt.Key_Less :
                    convkey = ','
                elif key == Qt.Key_Greater :
                    convkey = '.'
                elif key == Qt.Key_Space or key == Qt.Key_S :
                    convkey = 's'
                else:
                    convkey = 0

                # mstr = "key val : " + convkey
                # self.print_MessStr(mstr)

                # if convkey in moveBindings.keys():
                #     self.x = moveBindings[convkey][0]
                #     self.th = moveBindings[convkey][1]
                #     self.count = 0
                # elif convkey in speedBindings.keys():
                #     self.speed = self.speed * speedBindings[convkey][0]
                #     self.turn = self.turn * speedBindings[convkey][1]
                #     self.count = 0
                    
                #     mstr = self.vels(self.speed, self.turn)
                #     self.print_MessStr(mstr)

                # elif key == Qt.Key_Space or key == Qt.Key_S :
                #     self.x = 0
                #     self.th = 0
                #     self.control_speed = 0
                #     self.control_turn = 0
                # # else:
                # #     self.count = self.count + 1
                # #     if self.count > 4:
                # #         self.x = 0
                # #         self.th = 0
                        
                
                # self.target_speed = self.speed * self.x
                # self.target_turn = self.turn * self.th

                # if self.target_speed > self.control_speed:
                #     self.control_speed = min( self.target_speed, self.control_speed + 0.02 )
                # elif self.target_speed < self.control_speed:
                #     self.control_speed = max( self.target_speed, self.control_speed - 0.02 )
                # else:
                #     self.control_speed = self.target_speed

                # if self.target_turn > self.control_turn:
                #     self.control_turn = min( self.target_turn, self.control_turn + 0.1 )
                # elif self.target_turn < self.control_turn:
                #     self.control_turn = max( self.target_turn, self.control_turn - 0.1 )
                # else:
                #     self.control_turn = self.target_turn

                # twist = Twist()
                # twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
                # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
                # self.pub_twist.publish(twist)

                # sleep(0.1)

                # self.control_speed = 0
                # self.control_turn = 0

                # twist = Twist()
                # twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
                # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
                # self.pub_twist.publish(twist)


                # mstr = self.vels(self.control_speed, self.control_turn)
                # self.print_MessStr(mstr)

            else:

                if key == Qt.Key_W :
                    self.target_linear_vel = self.target_linear_vel + self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                    #print self.vels(self.target_linear_vel,self.target_angular_vel)            
                elif key == Qt.Key_X :
                    self.target_linear_vel = self.target_linear_vel - self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                    #print self.vels(self.target_linear_vel,self.target_angular_vel)
                elif key == Qt.Key_A :
                    self.target_angular_vel = self.target_angular_vel + self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                    #print self.vels(self.target_linear_vel,self.target_angular_vel)
                elif key == Qt.Key_D :
                    self.target_angular_vel = self.target_angular_vel - self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                    #print self.vels(self.target_linear_vel,self.target_angular_vel)
                elif key == Qt.Key_Space or key == Qt.Key_S :
                    self.target_linear_vel   = 0
                    self.control_linear_vel  = 0
                    self.target_angular_vel  = 0
                    self.control_angular_vel = 0
                    mstr = self.vels(0, 0)
                    self.print_MessStr(mstr)
                    #print self.vels(0, 0)            

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

            # teleop_bool = Bool()
            # teleop_bool.data = 1


    def onMotionCtrlBtn(self):
        global bRunMCtrl

        if self.bmotionCtrl == False:
            self.bmotionCtrl= True
            bRunMCtrl = True

            mctrl_th = Thread_TeleOP(self)
            mctrl_th.start()
            
            #mctrl_th = threading.Thread(target=self.MotionControl_callback)
            #mctrl_th.daemon = True
            

            self.motionctrl_PB.setText('Stop MCtrl')

            self.print_MessStr("Run motion control thread.")
            
        else:
            self.bmotionCtrl= False
            bRunMCtrl = False
            self.motionctrl_PB.setText('Motion Ctrl')
            self.print_MessStr("Stop motion control thread.")


    def onParkingChargingStationBtn(self):
        if self.bCSParking == False:
            self.bCSParking = True
            #os.system("python3 ~/bin/target_term -run 12 roslaunch autocharge parkingcs.launch")
            
            self.client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            self.client.wait_for_server()

            goal = ChargingGoal()
            self.client.send_goal(goal)

            self.print_MessStr("Send goal for parking charging station.")
            self.csparking_PB.setText('Cancel')

        else:
            self.bCSParking = False
            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)

            print(cmdstr)

            self.print_MessStr("Cancel parking charge station.")
            self.csparking_PB.setText('Parking C.S.')

            #client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            #client.wait_for_server()

            # while True:
            #     charging_result = self.client.get_result().result

            #     print("charging result : " + charging_result)

            #     if charging_result == "cancel_charging":
            #         print("====>csStatus : " + charging_result)
            #         self.print_MessStr("Complete cancel charging station.")

            #         break

            #     sleep(0.5)
            #     #sleep(0.1)


            sleep(5)

            twist = Twist()
            twist.linear.x = 0.04; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(16)

            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)


            # nc = NavigationControl()
            # nc.control = NavigationControl.START
            # nc.goal_name = "chargingpos"
            # self.ncpub.publish(nc)
            # mstr = "[WP] Start way point : " + nc.goal_name
            # self.print_MessStr(mstr)


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

        # self.dsrrequest_srv.trajnum = 100
        # self.dsrrequest_srv.setstatus = DSRobot._request_class.START

        # result = self.dsrservice_client(self.dsrrequest_srv)
        # print(result)
    

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

    def onWPPauseBtn(self):
        nc = NavigationControl()
        nc.control = NavigationControl.PAUSE
        nc.goal_name = self.wpCBStr
        self.ncpub.publish(nc)
        mstr = "Pause way point : " + self.wpCBStr
        self.print_MessStr(mstr)

    def onDSRTrajRunBtn(self):

        # if self.bDSRStart == False:
        #     self.bDSRStart = True

            # val = re.findall(r'\d+', self.dsrtrajCBStr)
            # val2 = map(int, val)
 
            # self.dsrrequest_srv.trajnum = val2[0]
            # self.dsrrequest_srv.setstatus = DSRobot._request_class.START

            # result = self.dsrservice_client(self.dsrrequest_srv)
            # print(result)
        
        zbdsr_traj = ZBDSRSetTraj()
        #val = np.chararray(3)
        #val[:] = 0
        #print("dsrtrajCBStr:" + self.dsrtrajCBStr)
        val = re.findall(r'\d+', self.dsrtrajCBStr)
        #print(val)
        val2 = map(int, val)
        #print(val2)

        self.cur_trajnum = val2[0]
        zbdsr_traj.trajnum = self.cur_trajnum
        zbdsr_traj.setstatus = ZBDSRSetTraj.START
        # val3 = val2[0]
        # print(val3)
        # zbdsr_traj.trajnum = val3

        #print("val:" + zbdsr_traj.trajnum)
    
        # if len(self.dsrtrajCBStr) == 13:
        #     val[0] = self.dsrtrajCBStr[12]
        # elif len(self.dsrtrajCBStr) == 14:
        #     val[0] = self.dsrtrajCBStr[12]
        #     val[1] = self.dsrtrajCBStr[13]

        # print("val:" + val)
    
        
        #zbdsr_traj.trajnum = int(val)

        self.dsrpub.publish(zbdsr_traj)

        #self.dsrtraj_startPB.setText('Stop')

        mstr = "Start DSR Robot Trajectory Num : " + str(zbdsr_traj.trajnum)
        #mstr = "Start DSR Robot Trajectory Num : ()\r", str(self.dsrrequest_srv.trajnum)
        self.print_MessStr(mstr)

        if self.bStopStart == True:
            self.bStopStart = False
            self.dsrtraj_stopstartPB.setText('Stop')

        # else:
        #     zbdsr_traj = ZBDSRSetTraj()
            
        #     self.bDSRStart = False
        #     zbdsr_traj.trajnum = 0
        #     zbdsr_traj.setstatus = ZBDSRSetTraj.STOP

        #     self.dsrpub.publish(zbdsr_traj)

        #     self.dsrtraj_startPB.setText('Start')

        #     # self.dsrrequest_srv.trajnum = 0
        #     # self.dsrrequest_srv.setstatus = DSRobot._request_class.STOP

        #     # result = self.dsrservice_client(self.dsrrequest_srv)            
        #     # print(result)

        #     mstr = "Stop DSR Robot Trajectory!"
        #     self.print_MessStr(mstr)

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
        
        #p = Popen("/usr/bin/gnome-terminal", stdin=PIPE)
        #p.communication("ls -al")
        #os.system("ls -al")
        #os.system("gnome-terminal -- ls -al")

    def onTrajStopBtn(self):
        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.trajCBStr
        self.ncpub.publish(nc)
        mstr = "Stop trajector : " + self.trajCBStr
        self.print_MessStr(mstr)

    def onTrajPauseBtn(self):
        nc = NavigationControl()
        nc.control = NavigationControl.PAUSE
        nc.goal_name = self.trajCBStr
        self.ncpub.publish(nc)
        mstr = "Pause trajector : " + self.trajCBStr
        self.print_MessStr(mstr)

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
        else:
            self.bBringUpRun = False      
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /STM")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /ioctrl")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /maxonmotor_idx_control_test_node /robot_state_publisher")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /robot_state_publisher")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /sick_safetyscanners_front")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /sick_safetyscanners_rear")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /stm_starter")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zetabank_diagnostics")            


            os.system("python3 ~/bin/target_term -run 10 rosnode kill /charging_act")

            #cmd = "python3 ~/bin/target_term -run 1 kill -9 " + self.t_term[0]
            #print("cmd:", cmd)
            #os.system(cmd)
            #os.system("python3 ~/bin/target_term -run 1 kill -9  SIGINT")
            #os.system("python3 ~/bin/target_term -run 1 kill -s  SIGINT")
            #print("t_term[0]:", self.t_term[0])
            #os.killpg(t_term[0], signal.SIGINT)
            #os.system("python3 ~/bin/target_term -run 1 exit")            
            #os.system("python3 ~/bin/target_term -run 1 kill -2 $$")
            self.print_MessStr("Exit zetabank_robot.launch")
            self.run_bringupPB.setText('Run Bringup')
        
    def onSlamBtn(self):
        if self.bSLAMRun is False:
            self.bSLAMRun = True
            os.system("python3 ~/bin/target_term -run 2 roslaunch zetabank_slam zetabank_slam.launch")
            #os.system("gnome-terminal -- roslaunch zetabank_slam zetabank_slam.launch")
            self.print_MessStr("Run roslaunch zetabank_slam.launch")
            self.run_slamPB.setText('Stop SLAM')
        else:
            self.bSLAMRun = False      
            #os.system("python3 ~/bin/target_term -run 2 kill -2 $$")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zetabank_slam_gmapping")                        
            self.print_MessStr("Exit roslaunch zetabank_slam.launch")
            self.run_slamPB.setText('Run SLAM')

    def onTeleopBtn(self):
        if self.bTeleOPRun is False:
            self.bTeleOPRun = True
            os.system("python3 ~/bin/target_term -run 3 roslaunch teleop_keyandjoy zetabank_teleop_key.launch")
            #os.system("gnome-terminal -- roslaunch teleop_keyandjoy zetabank_teleop_key.launch")
            self.print_MessStr("Run roslaunch zetabank_teleop_key.launch")
            self.run_teleopPB.setText('Stop teleop')
        else:
            #os.system("python3 ~/bin/target_term -run 3 kill -2 $$")                        
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
            #os.system("python3 ~/bin/target_term -run 4 kill -2 $$")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /laserscan_multi_merger")             
            self.print_MessStr("Exit roslaunch laserscan_multi_merger.launch")
            self.run_2LRFPB.setText('Act 2LRF')

    def onNavigationBtn(self):
        if self.bNaviRun is False:
            self.bNaviRun = True
            os.system("python3 ~/bin/target_term -run 5 roslaunch zetabank_navigation normal_navigation.launch")
            self.print_MessStr("Run roslaunch normal_navigation.launch")
            self.run_naviPB.setText('Stop Navi.')
        else:
            self.bNaviRun = False            
            #os.system("python3 ~/bin/target_term -run 5 kill -2 $$")   
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /amcl")             
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /map_server")
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /move_base")            
            self.print_MessStr("Exit roslaunch normal_navigation.launch")
            self.run_naviPB.setText('Run Navi.')

    def onRVIZBtn(self):
        if self.bRVIZRun is False:
            self.bRVIZRun = True
            
            cmdstr = "python3 ~/bin/target_term -run 6 rosrun rviz rviz -d " + rviz_fname
            os.system(cmdstr)
            
            # os.system("python3 ~/bin/target_term -run 6 rosrun rviz rviz")
            
            self.print_MessStr("Run rosrun rviz")
            self.run_rvizPB.setText('Stop RVIZ')
        else:
            self.bRVIZRun = False
            #cmd = "python3 ~/bin/target_term -run 1 kill -9 " + self.t_term[5]
            #print("cmd:", cmd)
            #os.system(cmd)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill 'rosnode list | grep rviz_*'")
            #rosnode kill $(rosnode list | grep rviz_*) or rosnode kill `rosnode list | grep rviz_*``
            #os.system("python3 ~/bin/target_term -run 6 kill -2 $$")   
            #os.system("python3 ~/bin/target_term -run 1 rosnode kill /amcl")
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
            #os.system("python3 ~/bin/target_term -run 7 kill -2 $$")   
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
            #os.system("python3 ~/bin/target_term -run 8 kill -2 $$")   
            self.print_MessStr("Stop roslaunch navigationWayPoint.launch")
            self.run_makeWPPB.setText('Run makeWP')

    def onSaveWPBtn(self):
        os.system("python3 ~/bin/target_term -run 10 rostopic pub save_wp std_msgs/String \"save\" -1")
        #os.system("gnome-terminal -- rostopic pub save_wp std_msgs/String \"save\" -1")
        self.print_MessStr("Run rostopic pub save_wp")

    def onDSRRobotBtn(self):
        if self.bDSRRobotRun is False:
            self.bDSRRobotRun = True
            os.system("python3 ~/bin/target_term -run 9 roslaunch zetabank_dsr zetabank_dsr.launch mode:=real host:=192.168.10.110 port:=12345 model:=m0609 color:=white")
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

            # rospy.wait_for_service("/control_dsr_traj")
            # self.dsrservice_client = rospy.ServiceProxy("/control_dsr_traj", DSRobot)
            # self.dsrrequest_srv = DSRobotRequest()
            # #result = self.dsrservice_client(self.dsrrequest_srv)

            # self.dsrServiceConOK = True
        else:
            self.bTraj1Run = False
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
            #ctrlval = (self.SelectWF & 0b01)
            #print(ctrlval)
            self.SelectWF = ((self.SelectWF & 0b11) | 0b01)

            #self.SelectWF = ((self.SelectWF & 0b11) & 0b10)
            print(self.SelectWF)

            self.frontwfcPB.setText("Enable Front WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Activate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        else:
            self.bFrontWF = False
            #ctrlval = (self.SelectWF & 0b01)
            #print(ctrlval)

            self.SelectWF = ((self.SelectWF & 0b11) & 0b10)
            
            #self.SelectWF = ((self.SelectWF & 0b11) | 0b01)
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
            #ctrlval = (self.SelectWF & 0b10)
            #print(ctrlval)
            
            self.SelectWF = ((self.SelectWF & 0b11) | 0b10)

            #self.SelectWF = ((self.SelectWF & 0b11) & 0b01)

            print(self.SelectWF)

            self.rearwfcPB.setText("Enable Rear WF")

            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            self.print_MessStr("Activate warning field of rear lidar.")
            self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        else:
            self.bRearWF = False
            #ctrlval = (self.SelectWF & 0b10)
            #print(ctrlval)
            
            self.SelectWF = ((self.SelectWF & 0b11) & 0b01)
            
            #self.SelectWF = ((self.SelectWF & 0b11) | 0b10)
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
                #ctrlval = (self.SelWFRange & 0b01)
                #print(ctrlval)
                self.SelWFRange = ((self.SelWFRange & 0b11) & 0b10)
                #self.SelWFRange = ((self.SelWFRange & 0b11) | 0b01)
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
                #self.SelWFRange = ((self.SelWFRange & 0b11) & 0b10)
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
                #ctrlval = (self.SelWFRange & 0b01)
                #print(ctrlval)
                self.SelWFRange = ((self.SelWFRange & 0b11) & 0b01)
                #self.bRearSWF = ((self.bRearSWF & 0b11) | 0b01)
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
                #self.bRearSWF = ((self.bRearSWF & 0b11) & 0b10)
                print(self.SelWFRange)

                self.rearselwfrnagePB.setText("Rear Range #0")

                cmdstr = "gnome-terminal -- rostopic pub /WarningFieldSelect std_msgs/UInt8 \"data: " + str(self.SelWFRange) + "\" -1 "
                print(cmdstr)
                os.system(cmdstr)
                self.print_MessStr("Select rear #0 warning field.")
                self.rran_lineEdit.setText("0")



    def onPumpOnPBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /pump std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Activate the pump.")
        self.pumpled_label.setPixmap(QtGui.QPixmap(ICON_BLUE_LED))
   
    def onPumpOffPBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /pump std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Deactiavte the pump.")
        self.pumpled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onSolOnPBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /sol std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Activate the solenoid valve.")
        self.solled_label.setPixmap(QtGui.QPixmap(ICON_ORANGE_LED))

    def onSolOffPBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /sol std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Deactiavte the solenoid valve.")
        self.solled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onUVCEndtoolOnBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the UVC of end effector.")
        self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))

    def onUVCEndtoolOffBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /uvc_endtool std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the UVC of end effector.")
        self.UVCEled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onUVCBottomOnBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /uvc_bottom std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the UVC of bottom.")
        self.UVCBled_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))

    def onUVCBottomOffBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /uvc_bottom std_msgs/Bool \"data: false\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the UVC of bottom.")
        self.UVCBled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onScrubberCtrlOnBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /ScrubberControl std_msgs/Bool \"data: true\" -1 "
        print(cmdstr)
        os.system(cmdstr)
        self.print_MessStr("Turn on the Scrubber.")
        self.Scrubled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

    def onScrubberCtrlOffBtn(self):
        cmdstr = "gnome-terminal -- rostopic pub /ScrubberControl std_msgs/Bool \"data: false\" -1 "
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
        #QCoreApplication.instance().quit


#app = QtGui.QApplication(sys.argv)
app = QApplication(sys.argv)
window = MyWindow()
window.show()
sys.exit(app.exec_())
