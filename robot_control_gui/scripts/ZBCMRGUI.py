#!/usr/bin/env python

import os, sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
from python_qt_binding import loadUi
from time import time
from time import sleep

import yaml
import rospy
from time import strftime
from time import localtime
import subprocess

from zetabank_msgs.msg import NavigationControl, NavigationControlStatus
import os
import numpy as np
import re
import math

import glob
import pickle

from zetabank_msgs.msg import ZBDSRSetTraj, ZBDSRTrajStatus
from zetabank_msgs.msg import RobotControlStatus
from autocharge.msg import ChargingAction, ChargingActionGoal, ChargingGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

import actionlib
import threading
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import std_msgs
from std_msgs.msg import Bool, String, UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from zetabot_main.msg import PowerControlMsgs
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose

import schedule
import subprocess
from shlex import shlex
from os import kill
from signal import alarm, signal, SIGALRM, SIGKILL, SIGTERM, SIGINT
from subprocess import PIPE, Popen
from pynput.keyboard import Key, Controller
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *

import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError

from zetabank_msgs.msg import ZBDSRMove

from zetabank_msgs.msg import DSRPos

import argparse

rospy.init_node('ZBCMRGUI_node')

navirviz_fname = "/home/zetabank/catkin_ws/src/zetabank_robot_control/zetabank_navigation/rviz/zetabank_nav.rviz"
slamrviz_fname = "/home/zetabank/catkin_ws/src/zetabank_robot_control/zetabank_slam/rviz/zetabank_slam.rviz"

rootdir = rospy.get_param('~root_dir')

# parser = argparse.ArgumentParser(description="Robot Control Program")
# parser.add_argument('--rootdir', type=str, help='root directory : robot control gui', default='/home/zetabank/catkin_ws/src/robot_control_gui')

# args = parser.parse_args()

# print("args:", args)

# rootdir = args.rootdir

# print('rootdir:', rootdir)

fname = rootdir + "/scripts/ZBCMR_gui.ui"
amclui_fname = rootdir + "/scripts/GetAMCLPos.ui"
airinfoui_fname = rootdir + "/scripts/AirInfo.ui"

yaml_fname = rootdir +"/config/HDIWP99.yaml" #kiria_office2

schedule_fname = rootdir + "/config/DSchedule.yml"


form_class = uic.loadUiType(fname)[0]
amcl_ui = uic.loadUiType(amclui_fname)[0]
airinfo_ui = uic.loadUiType(airinfoui_fname)[0]

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

bScheduleLoadOK = False

initSpeed = 0.1
initTurn = 0.15

open_airinfo_dlg = False

bRunSchedule = False

MATH_RAD2DEG = 57.2957795

path = os.getcwd()
print("path:" + path)

bRunCalib = False

cal_path = ""
cal_file = "./calibration/ProCamCalibration.pckl"


def vels(target_linear_vel, target_angular_vel):
    return "currently ==> linear vel:%s angular vel:%s" % (target_linear_vel,target_angular_vel)

bRunSchedule = False
HDSch_List = []
CRDSch_List = []

bCAMAdapt = False

bUseCAMAdj = False

log_fname = []
log_now_time = []
safety_info = 0
working_region = 0
working_mode = 0
robot_mode = 1
robot_pos = []
logfp = []
blogFileReady = False

logdata = { "now_time" : "2202-08-01 12:00:00",
            "safety_info" : 0x00,
            "working_region" : 0x00,
            "working_mode" : 0x00,
            "robot_mode" : 0x00,
            "robot_status" : "none",
            "pos_x" : 0.0,
            "pos_y" : 0.0,
            "pos_z" : 0.0

}


class Thread_CheckerBoard_CalibrationProc(QThread):
    VideoSignal1 = pyqtSignal(QImage)
    printmess = pyqtSignal(str)
    # printedit = pyqtSignal(str)

    def __init__(self, parent = None):
        global cal_path

        super(Thread_CheckerBoard_CalibrationProc, self).__init__(parent)

        self.CheckerBoard = (6,9)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.objpoints = []
        self.imgpoints = [] 

        self.objp = np.zeros((1, self.CheckerBoard[0] * self.CheckerBoard[1], 3), np.float32)
        self.objp[0,:,:2] = np.mgrid[0:self.CheckerBoard[0], 0:self.CheckerBoard[1]].T.reshape(-1, 2)
        prev_img_shape = None
       

    def run(self):
        global bRunCalib
        global cal_path
        global path

        if cal_path is "":
            cal_path = path + "/calibration"

        fname = cal_path + "/*.jpg"
        print("fname:" + fname)
        images = glob.glob(fname)
        filenum = len(images)
        print(images)
        print("image num : " + str(filenum))

        if filenum > 0:
            for iname in images:

                print("image name:" + iname)
                # self.printedit.emit(os.path.basename(iname))

                calimg = cv2.imread(iname)

                calimg_copy = calimg.copy()

                # rgbImage = cv2.cvtColor(calimg_copy, cv2.COLOR_BGR2RGB)
                # h1, w1, ch1 = rgbImage.shape
                # bytesPerLine1 = ch1 * w1
                # qt_image1 = QImage(rgbImage.data, w1, h1, bytesPerLine1, QImage.Format_RGB888)                
                # p1 = qt_image1.scaled(320, 219, Qt.KeepAspectRatio)
                # self.VideoSignal4.emit(p1)

                grayImage = cv2.cvtColor(calimg, cv2.COLOR_BGR2GRAY)

                ret, corners = cv2.findChessboardCorners(grayImage, self.CheckerBoard, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

                print("ret:" + str(ret))
    
                if ret == True:
                    self.objpoints.append(self.objp)
                    corners2 = cv2.cornerSubPix(grayImage, corners, (11,11), (-1,-1), self.criteria)
                    
                    self.imgpoints.append(corners2)

                    calimg = cv2.drawChessboardCorners(calimg, self.CheckerBoard, corners2, ret)

                    h2, w2, ch2 = calimg.shape
                    bytesPerLine2 = ch2 * w2
                    qt_image = QImage(calimg.data, w2, h2, bytesPerLine2, QImage.Format_RGB888)                
                    p = qt_image.scaled(640, 437, Qt.KeepAspectRatio)
                    self.VideoSignal1.emit(p)

                sleep(1)

            h,w = calimg.shape[:2]

            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, grayImage.shape[::-1], None, None)
            
            print("Camera matrix : \n")
            print(mtx)

            print("dist : \n")
            print(dist)

            print("rvecs : \n")
            print(rvecs)
                
            print("tvecs : \n")
            print(tvecs)
        
            # Save values to be used where matrix+dist is required, for instance for posture estimation
            # I save files in a pickle file, but you can use yaml or whatever works for you
            f = open('./calibration/ProCamCalibration.pckl', 'wb')
            pickle.dump((mtx, dist, rvecs, tvecs), f)
            f.close()
                
            # Print to console our success
            print('Calibration successful. Calibration file used: {}'.format('./calibration/ProCamCalibration.pckl'))

            bRunCalib = False
            self.printmess.emit("[R]:Done calibration...")

    bRunCalib = False
    
    

class GetAMCLPosDialog(QDialog, amcl_ui):
    set_item = pyqtSignal(int, str)

    def __init__(self, parent=None):
        QDialog.__init__(self, parent)
        self.setupUi(self)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

        self.amclpos_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.CallbackAMCLPos)

        self.getdata_PB.clicked.connect(self.onGetDataBtn)
        self.gap_exit_PB.clicked.connect(self.onExittBtn)

        self.set_item.connect(self.setTableItem)

    @pyqtSlot(int, str)
    def setTableItem(self, int, str):
        self.covval_tableWidget.setItem(int, 0, QTableWidgetItem(str))

    def CallbackAMCLPos(self, msg):
        px = msg.pose.pose.position.x
        self.posx_lineEdit.setText(str(px))
        py = msg.pose.pose.position.y
        self.posy_lineEdit.setText(str(py))
        pz = msg.pose.pose.position.z
        self.posz_lineEdit.setText(str(pz))

        print("px : %f py : %f pz: %f" %(px, py, pz))

        ox = msg.pose.pose.orientation.x
        self.orientx_lineEdit.setText(str(ox))
        oy = msg.pose.pose.orientation.y
        self.orienty_lineEdit.setText(str(oy))
        oz = msg.pose.pose.orientation.z
        self.orientz_lineEdit.setText(str(oz))
        ow = msg.pose.pose.orientation.w
        self.orientw_lineEdit.setText(str(ow))

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        ang = yaw*MATH_RAD2DEG
        self.orienta_lineEdit.setText(str(ang))

        cov_val = msg.pose.covariance
        print(cov_val)
        print(" cov len :" + str(len(cov_val)))

        for i in range(0, len(cov_val)):
            self.set_item.emit(i, str(cov_val[i]))

    def process_run(self, args, cwd = None, shell = False, kill_tree = True, timeout = -1, env = None):
        '''
        Run a command with a timeout after which it will be forcibly
        killed.
        '''
        class Alarm(Exception):
            pass

        def alarm_handler(signum, frame):
            raise Alarm

        out = ''
        err = ''

        p = Popen(args, shell = shell, cwd = cwd, stdout = PIPE, stderr = PIPE, env = env)
        if timeout != -1:
            signal(SIGALRM, alarm_handler)
            alarm(timeout)
        try:
            out, err = p.communicate()
            if timeout != -1:
                alarm(0)
        except Alarm:
            pids = [p.pid]
            if kill_tree:
                pids.extend(self.get_process_children(p.pid))
            for pid in pids:
                # process might have died before getting to this line
                # so wrap to avoid OSError: no such process
                try:
                    kill(pid, SIGKILL)
                except OSError:
                    pass
            return -9, out, err
            # return -9, '', ''
        return p.returncode, out, err

    def get_process_children(self, pid):
        p = Popen('ps --no-headers -o pid --ppid %d' % pid, shell = True, stdout = PIPE, stderr = PIPE)
        out, err = p.communicate()
        return [int(p) for p in out.split()]    

    class Alarm(Exception):
        pass

    def alarm_handler(self, signum, frame):
        raise self.Alarm

    def onGetDataBtn(self):

        cmd = ['timeout 5 rostopic echo amcl_pose']
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            outs, errs = process.communicate()
        
            number = re.findall('\d+\.\d+', outs)
            print(number)

            self.posx_lineEdit.setText(str(number[0]))
            self.posy_lineEdit.setText(str(number[1]))
            self.posz_lineEdit.setText(str(number[2]))

            self.orientx_lineEdit.setText(str(number[3]))
            self.orienty_lineEdit.setText(str(number[4]))
            self.orientz_lineEdit.setText(str(number[5]))
            self.orientw_lineEdit.setText(str(number[6]))

            orientation_list = [number[3], number[4], number[5], number[6]]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            ang = yaw*MATH_RAD2DEG
            self.orienta_lineEdit.setText(str(ang))

            for i in range(7, 43):
                self.covval_tableWidget.setItem(i-7, 0, QTableWidgetItem(str(number[i])))

        except:
            print("error : amcl_pose")

    def onExittBtn(self):
        self.amclpos_sub.unregister()
        print("Exit get amcl point data proc...")
        self.close()
        
        
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
     
     
class Thread_ConfRoomDisfectionProcess(QThread):
    navi_runstatstr = pyqtSignal(str)
    # dsrtraj_runstat = pyqtSignal(str)
    dsrtraj_runstatstr = pyqtSignal(str)
    setrunflag = pyqtSignal(bool)
    setapflag = pyqtSignal(bool)
    setfwf = pyqtSignal(bool)
    setbwf = pyqtSignal(bool)
    setuvlamp =  pyqtSignal(bool)
    setuvblamp =  pyqtSignal(bool)
    # setbtuvclamp = pyqtSignal(bool)
    setlaser = pyqtSignal(bool)
    setpcsbtn = pyqtSignal(bool)
    setpump = pyqtSignal(bool)
    setsol = pyqtSignal(bool)

    estopdata = pyqtSignal(int)
    navi_runstat = pyqtSignal(int, str)
    dsrtraj_runstat = pyqtSignal(int, int)
    csstatus = pyqtSignal(str)
    

    def __init__(self, parent = None):
        super(Thread_ConfRoomDisfectionProcess, self).__init__(parent)
        
        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)

        self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRSetTraj, queue_size = 10)

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

        # rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)

        # rospy.Subscriber("/status_sdr_traj", ZBDSRTrajStatus, self.CallbackRunTrajStatus)

        # rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)
        
        # rospy.Subscriber("/EmergencyStop", UInt8, self.Callback_EStop) 

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

        self.estopdata.connect(self.setEStop)
        self.navi_runstat.connect(self.setNaviStat)
        self.dsrtraj_runstat.connect(self.setDSRTraj)
        self.csstatus.connect(self.setCSStatus)
        

    @pyqtSlot(int)
    def setEStop(self, data):
        self.bEStop_Status = data
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

    @pyqtSlot(int, str)
    def setNaviStat(self, status, stat_descript):
        self.naviStatus = status

        if(self.prevnaviStatus != self.naviStatus):
            mstr = stat_descript
            print(mstr)

            print("naviStatus : "+str(self.naviStatus))

            self.prevnaviStatus = self.naviStatus
              
    @pyqtSlot(str)
    def setCSStatus(self, stat):
        if self.bCSParking == True:
            self.csStatus = stat

            mstr = "Charging Station Status : " + self.csStatus
            
    @pyqtSlot(int, int)
    def setDSRTraj(self, curstatus, trajnum):
        stat_str = "none"
        
        if curstatus == ZBDSRTrajStatus.IDLING:
            stat_str = "Idling"
            self.trajStatus = ZBDSRTrajStatus.IDLING
        elif curstatus == ZBDSRTrajStatus.RUNNING:
            stat_str = "Running"
            self.trajStatus = ZBDSRTrajStatus.RUNNING
        elif curstatus == ZBDSRTrajStatus.PAUSED:
            stat_str = "Paused"
            self.trajStatus = ZBDSRTrajStatus.PAUSED
        elif curstatus == ZBDSRTrajStatus.COMPLETED:
            stat_str = "Completed"
            self.trajStatus = ZBDSRTrajStatus.COMPLETED
        elif curstatus == ZBDSRTrajStatus.CANCELLED:
            stat_str = "Canceled"
            self.trajStatus = ZBDSRTrajStatus.CANCELLED
        elif curstatus == ZBDSRTrajStatus.START:
            stat_str = "Start"
            self.trajStatus = ZBDSRTrajStatus.START
        elif curstatus == ZBDSRTrajStatus.STOP:
            stat_str = "Stop"
            self.trajStatus = ZBDSRTrajStatus.STOP
        elif curstatus == ZBDSRTrajStatus.ESTOP:
            stat_str = "EStop"
            self.trajStatus = ZBDSRTrajStatus.ESTOP
        elif curstatus == ZBDSRTrajStatus.RESTART:
            stat_str = "Restart"
            self.trajStatus = ZBDSRTrajStatus.RESTART
            
        mstr = "[HDIP]TrajNum(" + str(trajnum) + "), Status:" + stat_str
        print(mstr)

        self.dsrtraj_runstatstr.emit(mstr)
        
    # def Callback_EStop(self, msg):
    #     self.bEStop_Status = msg.data
    #     #print("EStop Status : {}\r".format(self.bEStop_Status))
        
    #     if self.bEStop_Status != 1 and self.bEStop == False:
    #     # if self.bEStop_Status != 0 and self.bEStop == False:
    #         if self.bEStop_Status == 0:
    #             print("Detected Lidar Field\r")
    #         elif self.bEStop_Status == 3:
    #             print("Pushed E-Stop Button\r")
    #         elif self.bEStop_Status == 2:
    #             print("Detected Lidar Field and Pushed E-Stop Button\r")

    #         self.bEStop = True

    #     elif self.bEStop == True and self.bEStop_Status == 1:
    #     # elif self.bEStop == True and self.bEStop_Status == 0:
    #         print("Release E-stop/ Lidar detecting\r")
            
    #         self.bEStop = False
    
    
    def StopWaypoint(self):
        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.wpName
        self.ncpub.publish(nc)
        mstr = "[WP] Stop way point : " + self.wpName
        print(mstr)
        self.navi_runstatstr.emit(mstr)

    # def CallbackCSStatus(self, request):
    #     if self.bCSParking == True:
    #         self.csStatus = request.data

    #         mstr = "Charging Station Status : " + self.csStatus

    #         self.navi_runstat.emit(mstr)
        

    # def CallbackRunNaviCtrlStatus(self, request):
    #     self.naviStatus = request.status

    #     if(self.prevnaviStatus != self.naviStatus):
    #         mstr = request.status_description
    #         print(mstr)
            
    #         self.navi_runstat.emit(mstr)

    #         self.prevnaviStatus = self.naviStatus


    # def CallbackRunTrajStatus(self, request):

    #     stat_str = "none"
        
    #     if request.curstatus == ZBDSRTrajStatus.IDLING:
    #         stat_str = "Idling"
    #         self.trajStatus = ZBDSRTrajStatus.IDLING
    #     elif request.curstatus == ZBDSRTrajStatus.RUNNING:
    #         stat_str = "Running"
    #         self.trajStatus = ZBDSRTrajStatus.RUNNING
    #     elif request.curstatus == ZBDSRTrajStatus.PAUSED:
    #         stat_str = "Paused"
    #         self.trajStatus = ZBDSRTrajStatus.PAUSED
    #     elif request.curstatus == ZBDSRTrajStatus.COMPLETED:
    #         stat_str = "Completed"
    #         self.trajStatus = ZBDSRTrajStatus.COMPLETED
    #     elif request.curstatus == ZBDSRTrajStatus.CANCELLED:
    #         stat_str = "Canceled"
    #         self.trajStatus = ZBDSRTrajStatus.CANCELLED
    #     elif request.curstatus == ZBDSRTrajStatus.START:
    #         stat_str = "Start"
    #         self.trajStatus = ZBDSRTrajStatus.START
    #     elif request.curstatus == ZBDSRTrajStatus.STOP:
    #         stat_str = "Stop"
    #         self.trajStatus = ZBDSRTrajStatus.STOP
    #     elif request.curstatus == ZBDSRTrajStatus.ESTOP:
    #         stat_str = "EStop"
    #         self.trajStatus = ZBDSRTrajStatus.ESTOP
    #     elif request.curstatus == ZBDSRTrajStatus.RESTART:
    #         stat_str = "Restart"
    #         self.trajStatus = ZBDSRTrajStatus.RESTART

    #     mstr = "TrajNum(" + str(request.trajnum) + "), Status:" + stat_str
    #     print(mstr)
        
    #     self.dsrtraj_runstat.emit(mstr)
        

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
        self.wait()
        
    def setRunFlag(self, val):
        self.bRunFlag = val
        
    def setPauseFlag(self, val):
        self.bRunPauseFlag = val

    # def PumpOn(self):
    #     cmdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 5 'true' -1 "
    #     # cmdstr = "gnome-terminal -- rostopic pub /pump std_msgs/Bool \"data: true\" -1 "
    #     print(cmdstr)
    #     os.system(cmdstr)
   
    # def PumpOff(self):
    #     mdstr = "gnome-terminal -- rostopic pub /power_control_command zetabot_main/PowerControlMsgs 5 'false' -1 "
    #     # cmdstr = "gnome-terminal -- rostopic pub /pump std_msgs/Bool \"data: false\" -1 "
    #     print(cmdstr)
    #     os.system(cmdstr)

    # def SolOn(self):
    #     cmdstr = "gnome-terminal -- rostopic pub /sol std_msgs/Bool \"data: true\" -1 "
    #     print(cmdstr)
    #     os.system(cmdstr)

    # def SolOff(self):
    #     cmdstr = "gnome-terminal -- rostopic pub /sol std_msgs/Bool \"data: false\" -1 "
    #     print(cmdstr)
    #     os.system(cmdstr)

    def CheckEStop(self):

        if self.bEStop == True:

            while True:

                if self.bEStop == False:
                    break

                sleep(0.2)

    def CheckAborted(self, wp):
        if self.naviStatus == NavigationControlStatus.ABORTED:
            self.navi_runstatstr.emit("Navi. Aborted")

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
        global CRDSch_List

        runcnt = 0

        bErrFlag = False

        self.setlaser.emit(True)

        # self.setuvblamp.emit(True)
        
        # proc_num = 0

        print("Start CRDI Navi Process... ")

        print("CRDSch_List ===>", CRDSch_List)
        
        for slist in CRDSch_List:
            print("Inside CRDSch_List routine")

            if self.bRunFlag == False:
                print("Stopping Navi Process... ")
                break

            proc_name = slist["name"] 
            proc_type = slist["type"]
            proc_sp = slist["start_point"]
            proc_ftnum = slist["dsrftnum"]
            proc_btnum = slist["dsrbtnum"]
            proc_wpump = slist["water_pump"]
            proc_uvlamp = slist["uvlamp"]

            print("----------------------")
            print("name : " + proc_name)
            print("type : " + proc_type)
            print("start point : " + proc_sp)
            print("dsrftnum : " + str(proc_ftnum))
            print("dsrbtnum : " + str(proc_btnum))
            print("water_pump : " + str(proc_wpump))
            print("uvlamp : " + str(proc_uvlamp))
            print("----------------------")   

            print("[1]type : " + proc_type)
            
            if proc_type == "waypoint":
                # =========================================
                # Conference Room Disinfection Process #1
                # =========================================  
                
                if self.bRunFlag == True:

                    self.setuvblamp.emit(True)
    
                    self.naviStatus = NavigationControlStatus.IDLING
                    
                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                    
                    print("[2]start point : " + proc_sp)

                    self.gotoWayPoint(proc_sp)

                    sleep(5)

                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue

                        self.CheckAborted(proc_sp)
                            
                        if self.bRunFlag == False:
                            self.StopWaypoint()
                            
                            self.navi_runstatstr.emit("Stop " + proc_sp + " CSDI process...")

                            self.setrunflag.emit(True)

                            sleep(1)
                            
                            self.stop()

                            break
                            
                        if self.bRunPauseFlag == True:
                            self.navi_runstatstr.emit("Pause " + proc_sp + " CSDI process...")
                            while self.bRunPauseFlag:
                                if self.bRunPauseFlag is False:
                                    self.navi_runstatstr.emit("Restart " + proc_sp + " CSDI process...")
                                    break
                            
                                sleep(0.1)

                            
                        if self.naviStatus == NavigationControlStatus.COMPLETED:
                            # self.CRDIRun[0] = 1
                            
                            print("Arrived at " + proc_sp + "...")
                            break
                            
                        if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                            runcnt = runcnt + 1

                            if runcnt <= self.RepCnt:
                                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                                self.gotoWayPoint(proc_sp)

                                self.navi_runstatstr.emit("Restart " + proc_sp)

                                self.naviStatus = NavigationControlStatus.IDLING

                            else:
                                # self.CRDIRun[0] = 0

                                runcnt = 0

                                self.navi_runstatstr.emit("Error(WARNNRGTO) " + proc_sp)

                                break

                        sleep(0.1)    
                        
            elif proc_type == "readydsr":
                # =========================================
                # Conference Room Disinfection Process #2
                # =========================================  
                if self.bRunFlag == True:
    
                    runcnt = 0

                    self.setuvlamp.emit(proc_uvlamp)

                    self.naviStatus = NavigationControlStatus.IDLING
                    
                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                    
                    self.gotoWayPoint(proc_sp)

                    sleep(5)

                    
                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue

                        self.CheckAborted(proc_sp)
                            
                        if self.bRunFlag == False:
                            self.StopWaypoint()

                            self.setrunflag.emit(True)

                            sleep(1)
                            
                            self.stop()

                            break
                            
                        if self.bRunPauseFlag == True:
                            self.navi_runstatstr.emit("Pause " + proc_sp + " CSDI process...")
                            while self.bRunPauseFlag:
                                if self.bRunPauseFlag is False:
                                    self.navi_runstatstr.emit("Restart " + proc_sp + " CSDI process...")
                                    break
                            
                                sleep(0.1)
                    
                        if self.naviStatus == NavigationControlStatus.COMPLETED:
                            # self.CRDIRun[1] = 1
                            
                            print("Arrived at " + proc_sp + "...")
                            break
                            
                        if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                            runcnt = runcnt + 1

                            if runcnt <= self.RepCnt:
                                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                                self.gotoWayPoint(proc_sp)

                                self.navi_runstatstr.emit("Restart " + proc_sp)

                                self.naviStatus = NavigationControlStatus.IDLING

                            else:
                                # self.CRDIRun[1] = 0

                                runcnt = 0

                                self.navi_runstatstr.emit("Error(WARNNRGTO) " + proc_sp)

                                break

                        sleep(0.1)

                    
                    if self.bRunFlag == True:
                        self.trajStatus = ZBDSRTrajStatus.IDLING
                        sleep(2)
                        
                        # 2022. 09. 05
                        self.setuvlamp.emit(proc_uvlamp)
                        # self.setbtuvclamp.emit(True)
                        # sleep(1.0)

                        self.RunDSRTraj(proc_ftnum)

                        sleep(12)

                        print("Moving Trjaectory ...")

                        print("[" + proc_sp + "] Send topic [Run Trajectory]")

                        while True:
                            if self.bEStop == True:
                                sleep(0.1)
                                continue

                            if self.bEStop == False and self.bprevEStop == True:
                                print("====> delay 5 sec...")
                                sleep(5)
                                self.bprevEStop = False
                            
                            if self.bRunFlag == False:
                                self.stopTraj()

                                self.dsrtraj_runstatstr.emit("Stop moving CRDI trajectory...")
                                # self.dsrtraj_runstat.emit("Stop moving CRDI trajectory...")
                                    
                                sleep(1)
                                    
                                self.stop()

                                break

                            if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                                
                                break

                            sleep(0.1)


                        print("Complete moving Trjaectory...")
                        
            elif proc_type == "wateron":
                # =========================================
                # Conference Room Disinfection Process #3
                # =========================================  
                if self.bRunFlag == True:
                    #if bErrFlag == False and self.bRunFlag == True:
                    runcnt = 0

                    self.setuvblamp.emit(proc_uvlamp)

                    self.naviStatus = NavigationControlStatus.IDLING
                    
                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                    
                    self.gotoWayPoint(proc_sp)

                    sleep(5)
        
                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue

                        self.CheckAborted(proc_sp)
                            
                        if self.bRunFlag == False:
                            self.StopWaypoint()
                            
                            self.navi_runstatstr.emit("Stop " + proc_sp + " CSDI process...")

                            self.setrunflag.emit(True)

                            sleep(1)
                            
                            self.stop()

                            break
                            
                        if self.bRunPauseFlag == True:
                            self.navi_runstatstr.emit("Pause " + proc_sp + " CSDI process...")
                            while self.bRunPauseFlag:
                                if self.bRunPauseFlag is False:
                                    self.navi_runstatstr.emit("Restart " + proc_sp + " CSDI process...")
                                    break
                            
                                sleep(0.1)
                    
                        if self.naviStatus == NavigationControlStatus.COMPLETED:
                            # self.CRDIRun[1] = 1
                            
                            print("Arrived at " + proc_sp + "...")
                            break
                            
                        if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                            runcnt = runcnt + 1

                            if runcnt <= self.RepCnt:
                                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                                self.gotoWayPoint(proc_sp)

                                self.navi_runstatstr.emit("Restart " + proc_sp)

                                self.naviStatus = NavigationControlStatus.IDLING

                            else:
                                # self.CRDIRun[1] = 0

                                runcnt = 0

                                self.navi_runstatstr.emit("Error(WARNNRGTO) " + proc_sp)

                                break

                        sleep(0.1)        

                    if self.bRunFlag == True:
                        # self.PumpOn()
                        self.setpump.emit(proc_wpump)

                        print("Turn on Pump...")
                        
                        sleep(1)

                        self.setsol.emit(proc_wpump)
                        # self.SolOn()
                        
                        print("Turn on Sol. valve...")

                        # if proc_wpump == True:
                        #     # self.PumpOn()
                        #     self.setpump.emit(True)

                        #     print("Turn on Pump...")
                            
                        #     sleep(1)

                        #     self.setsol.emit(True)
                        #     # self.SolOn()
                            
                        #     print("Turn on Sol. valve...")
                        
            elif proc_type == "wateroff":
                # =========================================
                # Conference Room Disinfection Process #4
                # =========================================  
                if self.bRunFlag == True:
                    runcnt = 0

                    self.setuvblamp.emit(proc_uvlamp)

                    self.naviStatus = NavigationControlStatus.IDLING
                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                    self.gotoWayPoint(proc_sp)

                    sleep(5)
        
                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue

                        self.CheckAborted(proc_sp)
                            
                        if self.bRunFlag == False:
                            self.StopWaypoint()
                            
                            self.navi_runstatstr.emit("Stop " + proc_sp + " CSDI process...")

                            self.setrunflag.emit(True)

                            sleep(1)
                            
                            self.stop()

                            break
                            
                        if self.bRunPauseFlag == True:
                            self.navi_runstatstr.emit("Pause " + proc_sp + " CSDI process...")
                            while self.bRunPauseFlag:
                                if self.bRunPauseFlag is False:
                                    self.navi_runstatstr.emit("Restart " + proc_sp + " CSDI process...")
                                    break
                            
                                sleep(0.1)
                        
                        if self.naviStatus == NavigationControlStatus.COMPLETED:
                            # self.CRDIRun[2] = 1              
                            
                            print("Arrived at " + proc_sp + "...")
                            break
                            
                        if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                            runcnt = runcnt + 1

                            if runcnt <= self.RepCnt:
                                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                                self.gotoWayPoint(proc_sp)

                                self.navi_runstatstr.emit("Restart " + proc_sp)

                                self.naviStatus = NavigationControlStatus.IDLING

                            else:
                                # self.CRDIRun[2] = 0

                                runcnt = 0

                                self.navi_runstatstr.emit("Error(WARNNRGTO) " + proc_sp)

                                break

                        sleep(0.1)

                    if self.bRunFlag == True:     
                        self.setpump.emit(proc_wpump)
                        # self.PumpOff()
                        print("Turn off Pump...")

                        sleep(1)

                        self.setsol.emit(proc_wpump)                            
                        # self.SolOff()
                        print("Turn off Sol. valve...")

                        # if proc_wpump == False:
                        #     self.setpump.emit(False)
                        #     # self.PumpOff()
                        #     print("Turn off Pump...")

                        #     sleep(1)

                        #     self.setsol.emit(False)                            
                        #     # self.SolOff()
                        #     print("Turn off Sol. valve...")
                        
            elif proc_type == "stopdsr":
                # =========================================
                # Conference Room Disinfection Process #5
                # =========================================  
                if self.bRunFlag == True:
                    runcnt = 0

                    # self.setuvblamp.emit(proc_uvlamp)

                    self.naviStatus = NavigationControlStatus.IDLING
                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                    self.gotoWayPoint(proc_sp)

                    sleep(5)

                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue

                        self.CheckAborted(proc_sp)
                            
                        if self.bRunFlag == False:
                            self.StopWaypoint()
                            
                            self.navi_runstatstr.emit("Stop " + proc_sp + " process...")

                            self.setrunflag.emit(True)

                            sleep(1)
                            
                            self.stop()

                            break
                            
                        if self.bRunPauseFlag == True:
                            self.navi_runstatstr.emit("Pause " + proc_sp + " process...")
                            while self.bRunPauseFlag:
                                if self.bRunPauseFlag is False:
                                    self.navi_runstatstr.emit("Restart " + proc_sp + " CSDI process...")
                                    break
                            
                                sleep(0.1)
                    
                        if self.naviStatus == NavigationControlStatus.COMPLETED:
                            # self.CRDIRun[3] = 1
                            
                            print("Arrived at " + proc_sp + "...")
                            break
                            
                        if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                            runcnt = runcnt + 1

                            if runcnt <= self.RepCnt:
                                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                                self.gotoWayPoint(proc_sp)

                                self.navi_runstatstr.emit("Restart " + proc_sp)

                                self.naviStatus = NavigationControlStatus.IDLING

                            else:
                                # self.CRDIRun[3] = 0

                                runcnt = 0

                                self.navi_runstatstr.emit("Error(WARNNRGTO) " + proc_sp)

                                break

                        sleep(0.1)

                        
                    if self.bRunFlag == True:
                        
                        self.setuvlamp.emit(proc_uvlamp)
                        # self.setuvlamp.emit(False)
                        # self.setbtuvclamp.emit(False)
                        sleep(1.0)

                        zbdsr_traj = ZBDSRSetTraj()            
                        zbdsr_traj.trajnum = proc_ftnum
                        zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                        self.dsrpub.publish(zbdsr_traj)

                        sleep(2)

                        self.trajStatus = ZBDSRTrajStatus.IDLING
                        #sleep(2)

                        self.RunDSRTraj(proc_btnum)

                        print("[" + proc_sp + "] Send topic [Run Trajectory]")

                        while True:
                            if self.bEStop == True:
                                sleep(0.1)
                                continue

                            if self.bEStop == False and self.bprevEStop == True:
                                print("====> delay 5 sec...")
                                sleep(5)
                                self.bprevEStop = False
                            
                            if self.bRunFlag == False:
                                self.stopTraj()

                                self.dsrtraj_runstatstr.emit("Stop moving CRDI trajectory...")
                                # self.dsrtraj_runstat.emit("Stop moving CRDI trajectory...")
                                    
                                sleep(1)
                                    
                                self.stop()

                                break

                            if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                                
                                break

                            sleep(0.1)


                        print("Complete moving Trjaectory...")
                        
            elif proc_type == "parking":
                # =========================================
                # Conference Room Disinfection Process #6
                # =========================================  
                if self.bRunFlag == True:
                    self.naviStatus = NavigationControlStatus.IDLING

                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
    
                    client = actionlib.SimpleActionClient('charging_act', ChargingAction)
                    client.wait_for_server()

                    self.setlaser.emit(False)

                    goal = ChargingGoal()
                    client.send_goal(goal)

                    print("Parking charging station...")
                    
                    self.bCSParking = True

                    bStartCharging = False

                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue
                            
                        if self.bRunFlag == False:
                            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
                            os.system(cmdstr)

                            self.navi_runstatstr.emit("Stop auto charging process...")
                            
                            # self.StopWaypoint()
                            
                            # self.navi_runstat.emit("Stop auto parking CSDI process...")

                            # self.setrunflag.emit(True)

                            sleep(1)
                            
                            self.stop()

                            break
                        
                        if self.csStatus == "search" and bStartCharging == False:
                            bStartCharging = True
                            print("Arrived at charging station position...")  
                            print("====>csStatus : " + self.csStatus)
                            self.navi_runstatstr.emit("Searching.....")

                            self.setlaser.emit(False)

                            self.setuvlamp.emit(proc_uvlamp)

                            sleep(2)
                            self.setuvblamp.emit(proc_uvlamp)
                            # self.setbtuvclamp.emit(False)

                        if self.csStatus == "contact":
                            print("====>csStatus : " + self.csStatus)
                            self.navi_runstatstr.emit("Complete parking charging station.")

                            self.setapflag.emit(True)
                            # self.setrunflag.emit(False)  
                            
                            self.setpcsbtn.emit(False)  
                            print("change Parking C.S button...")                   

                            break
                            
                        sleep(0.1)
                
                    if self.bRunFlag == True:
                        print("Complete all disinfection processes in the conference room.")
            
                else:
                    print("Stop CSDI processing!!!")

                self.stop()

    def run(self):
        for i in range(1, 15):
            self.CRDIRun[i] = 0

        if self.bRunFlag == True:
            self.navi_Process()

        self.stop()

    def stop(self):

        self.quit()
        self.wait()


            
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
        self.wait()
        
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
                    sleep(0.1)
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
                    sleep(0.1)
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

                self.RunDSRTraj(7)

                sleep(10)

                print("Moving Trjaectory #7...")

                # while True:
                #     if self.bEStop == True:
                #         sleep(0.1)
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
                    sleep(0.1)
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
                # self.PumpOn()

                print("Turn on Pump...")
                
                sleep(1)

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
                if self.bEStop == True:
                    sleep(0.1)
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
                if self.bEStop == True:
                    sleep(0.1)
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
                    sleep(0.1)
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
                    sleep(0.1)
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
                # self.PumpOn()

                print("Turn on Pump...")
                
                sleep(1)

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
                if self.bEStop == True:
                    sleep(0.1)
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
                if self.bEStop == True:
                    sleep(0.1)
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
                    sleep(0.1)
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
                    sleep(0.1)
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
                # self.PumpOn()

                print("Turn on Pump...")
                
                sleep(1)

                # self.SolOn()
                
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
                    sleep(0.1)
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
                # self.SolOff()
                
                print("Turn off Sol. valve...")
                
                # self.PumpOff()

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
                    sleep(0.1)
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
                    sleep(0.1)
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
                # self.PumpOn()

                print("Turn on Pump...")
                
                sleep(1)

                # self.SolOn()
                
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
                    sleep(0.1)
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
                # self.SolOff()
                
                print("Turn off Sol. valve...")
                
                # self.PumpOff()

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
                    sleep(0.1)
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

                self.RunDSRTraj(8)

                print("[CRWP3] Send topic [Run Trajectory 6]")

                while True:
                    if self.bEStop == True:
                        sleep(0.1)
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
        #             sleep(0.1)
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
        #                 sleep(0.1)
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
                    sleep(0.1)
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
                    sleep(0.1)
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

                    # self.HDIRun[5] = 0

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
                    sleep(0.1)
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



class Thread_HallowDisfectionProcess(QThread):
    navi_runstatstr = pyqtSignal(str)
    # navi_runstat = pyqtSignal(str)
    # dsrtraj_runstat = pyqtSignal(str)
    dsrtraj_runstatstr = pyqtSignal(str)
    setrunflag = pyqtSignal(bool)
    setapflag = pyqtSignal(bool)
    setfwf = pyqtSignal(bool)
    setbwf = pyqtSignal(bool)
    setuvlamp =  pyqtSignal(bool)
    setuvblamp =  pyqtSignal(bool)
    setlaser = pyqtSignal(bool)
    setpcsbtn = pyqtSignal(bool)

    VideoSignal2 = pyqtSignal(QImage)
    VideoSignal3 = pyqtSignal(QImage)
    
    print_mess = pyqtSignal(str)
    clearscene = pyqtSignal(int)
    # setmovbtn = pyqtSignal(bool)
    endadj = pyqtSignal()
    estopdata = pyqtSignal(int)
    navi_runstat = pyqtSignal(int, str)
    dsrtraj_runstat = pyqtSignal(int, int)
    csstatus = pyqtSignal(str)

    pubtwist = pyqtSignal(float)
    # setmovbtn = pyqtSignal(bool)


    def __init__(self, parent = None):
        super(Thread_HallowDisfectionProcess, self).__init__(parent)
        
        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)
        self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRSetTraj, queue_size = 10)
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)        
        self.rcspub = rospy.Publisher("robot_control_status", RobotControlStatus, queue_size = 10)
        self.pub_movpos = rospy.Publisher("/control_dsr_mov", ZBDSRMove, queue_size = 10)
        self.senddsrpos_sub = rospy.Subscriber("/send_dsr_posx", DSRPos, self.CallSendPos) 
        self.pub_getdsrpos = rospy.Publisher("/get_dsr_posx", Bool, queue_size = 10)

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

        # 2022. 08. 08
        # self.cvImg = None
        self.grayImg = None
        self.rotImg = None
        self.flipImg = None
        self.qtImg = None
        self.scaledImg = None
        self.originImg = None
        # self.bGetImgOK = False
        self.bImgProcOK = False
        self.bAdjustPos = False
        self.bMoving = False
        # self.bCAMAdapt = False

        self.movedir = 0
        self.mov_gainy = 0.2
        self.mov_gainz = 0.3
        self.diffconstx = 20.0
        self.diffconsty = 25.0
        self.diffangle = 3.0
        self.initangle = 15.0
        self.fmoveval1 = 230.0
        # self.fmoveval1 = 220.0
        self.fmoveval2 = 70.0
        self.finaloffset = -10.0
        self.movdiff = 20.0
        # self.movdiff = 40.0
        self.angleAdjCnt = 0
        self.AdjStep = 1
        self.readAngCnt = 0
        self.getPosVal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.area = 0
        self.bGetPosOk = False
        self.diffx = 0
        self.diffy = 0
        self.prev_diffx = 0
        self.prev_diffy = 0
        self.movcnt = 100
        self.divangle = 3.0
        self.repcnt = 0

        self.estop_movdir = 0

        self.trajStatus = ZBDSRTrajStatus.IDLING
        self.cvBridge = CvBridge()

        self.bCAMProcEnd = False

        self.StartCnt = 0
        self.bReadyCAMAdj = False

        self.estopdata.connect(self.setEStop)
        self.navi_runstat.connect(self.setNaviStat)
        self.dsrtraj_runstat.connect(self.setDSRTraj)
        self.csstatus.connect(self.setCSStatus)

        vindex = self.video_detect_index()
        print("video device index : ", vindex)

        # 2022. 08. 25
        self.cap = cv2.VideoCapture(vindex)

    def video_detect_index(self):
        video_idx = None

        for video_idx_file in os.listdir("/sys/class/video4linux"):
            video_file_path = os.path.realpath("/sys/class/video4linux/" + video_idx_file)

            for video_files in os.listdir(video_file_path):
                if 'name' in video_files:
                    name = open(os.path.realpath(video_file_path + '/' + 'name'), 'r').readline()
                    print("name:" + name)

                    if re.match('FULL HD', name) != None:
                        index = open(os.path.realpath(video_file_path + '/' + 'index'), 'r').readline()
                        print("index:")
                        print(index)
                        if re.match('0', index) != None:
                            video_idx = "/dev/" + video_idx_file
                            print(video_idx)        

        return video_idx
        

    @pyqtSlot(int)
    def setEStop(self, data):
        self.bEStop_Status = data
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

    def CallSendPos(self, msg):

        self.getPosVal[0] = msg.px
        self.getPosVal[1] = msg.py
        self.getPosVal[2] = msg.pz
        self.getPosVal[3] = msg.rx
        self.getPosVal[4] = msg.ry
        self.getPosVal[5] = msg.rz

        # print("getPosval:" + str(self.getPosVal))

        self.bGetPosOk = True

    def SetRunFlag(self, val):
        self.bRunFlag = val

    def SetAdjustFlag(self, val):
        self.bAdjustPos = val

    # def SetCAMAdapt(self, val):
    #     self.bCAMAdapt = val

    def RunRobotMove(self):
        zbdsr_move = ZBDSRMove()

        zbdsr_move.xpos = 661.23
        zbdsr_move.ypos = -0.31 + 50
        zbdsr_move.zpos = 499.15 + 50
        zbdsr_move.rxpos = 1.53
        zbdsr_move.rypos = 91.58
        zbdsr_move.rzpos = 180.0
        zbdsr_move.setstatus = ZBDSRMove.START

        self.pub_movpos.publish(zbdsr_move)

        mstr = "Start DSR Robot Moving Pos #1"
        self.print_mess.emit(mstr)

        while True:

            if self.trajStatus == ZBDSRMove.COMPLETED:
                break

            sleep(0.1)

        zbdsr_move = ZBDSRMove()

        zbdsr_move.xpos = 661.23
        zbdsr_move.ypos = -0.31 - 100
        zbdsr_move.zpos = 499.15
        zbdsr_move.rxpos = 1.53
        zbdsr_move.rypos = 91.58
        zbdsr_move.rzpos = 180.0
        zbdsr_move.setstatus = ZBDSRMove.START

        self.pub_movpos.publish(zbdsr_move)

        mstr = "Start DSR Robot Moving Pos #2"
        self.print_mess.emit(mstr)

        while True:

            if self.trajStatus == ZBDSRMove.COMPLETED:
                break

            sleep(0.1)

        mstr = "End Moving Pos of DSR Robot..."
        self.print_mess.emit(mstr)

    def runCAMAdjProcess(self):
        self.bCAMProcEnd = False

        cx = 0
        cy = 0
        self.diffx = 0
        self.diffy = 0

        # 2022. 08. 12
        prev_xpos = 487.03
        prev_ypos = -44.540
        prev_zpos = 525.39
        prev_rxpos = 179.14
        prev_rypos = -93.07
        prev_rzpos = 0.2
        angle = 0
        self.AdjStep = 1
        self.caperr_cnt = 0

        self.repcnt = 0
        self.bMobiling = False

        self.bImgProcOK = False

        print("running CAMAdjProcess...")

        # cmdstr = "gnome-terminal -- rostopic pub /get_dsr_posx std_msgs/Bool \"data: true\" -1 "
        # os.system(cmdstr)

        sval = Bool()
        sval.data = True
        self.pub_getdsrpos.publish(sval)

        self.bEStop = False
        self.bprevEStop = False
        self.bMoving = False

        # self.setmovbtn.emit(True)

        self.StartCnt = 0
        self.bReadyCAMAdj = False
        
        while True:
            
            if self.bCAMProcEnd is True:
                break

            if self.bRunFlag == False:
                self.stopTraj()

                self.navi_runstatstr.emit("Stop CAM Adj(HDI) process...")

                break


            # self.bGetImgOK = True
            # print("EStop :", self.bEStop, " prev EStop:", self.bprevEStop)

            # 2022. 08. 26
            if self.bprevEStop == False and self.bEStop == True:
                self.pubtwist.emit(0.0)
                sleep(0.5)
                self.pubtwist.emit(0.0)
                print("CAM Adj : Set EStop")

                self.bprevEStop = self.bEStop

            if self.bprevEStop == True and self.bEStop == False:
                self.bprevEStop = self.bEStop

                sleep(4)

                print("CAM Adj : Reset EStop")
                if self.AdjStep == 1:
                    self.pubtwist.emit(0.03)
                    print("[S1 Release EStop] : forward x-axis moving...")
                elif self.AdjStep == 2:
                    if self.estop_movdir == 0:
                        self.pubtwist.emit(0.03) 
                        print("[S2 Release EStop] : forward x-axis moving...")
                    elif self.estop_movdir == 1:
                        self.pubtwist.emit(-0.03) 
                        print("[S2 Release EStop] : backward x-axis moving...")

                elif self.AdjStep == 4:
                    if self.diffx < -1.0*self.movdiff and self.prev_diffx >= 0:
                        self.pubtwist.emit(0.02)
                        self.prev_diffx = self.diffx
                        print("[S4 Release EStop] : Exceed forward x-axis moving...")
                        sleep(1)
                    elif self.diffx > self.movdiff and self.prev_diffx <= 0:
                        self.pubtwist.emit(-0.02)
                        self.prev_diffx = self.diffx
                        print("[S4 Release EStop] : Exceed backward x-axis moving...")
                        sleep(1)
                elif  self.AdjStep == 5 or self.AdjStep == 6:
                    zbdsr_move = ZBDSRMove()
                    zbdsr_move.xpos = prev_xpos                    
                    zbdsr_move.ypos = prev_ypos                    
                    zbdsr_move.zpos = prev_zpos
                    zbdsr_move.rxpos = prev_rxpos
                    zbdsr_move.rypos = prev_rypos
                    zbdsr_move.rzpos = prev_rzpos
                    zbdsr_move.setstatus = ZBDSRMove.START
                    self.pub_movpos.publish(zbdsr_move)
                    
                    self.trajStatus = ZBDSRTrajStatus.IDLING
                    print("[S{} Release EStop] : Restart moving...".format(self.AdjStep))

                    self.bMoving = True



            if self.bImgProcOK == False:
            # if self.bGetImgOK == True and self.bImgProcOK == False:
                self.bImgProcOK = True

                # 2022. 08. 25
                # ret, self.cvImg = self.cap.read()
                # 2022. 09. 03
                ret, cvImg = self.cap.read()

                # print("read image")

                if ret is False:
                    print("can't open video!!")

                    self.caperr_cnt += 1

                    if self.caperr_cnt > 10:
                        self.caperr_cnt = 0

                        vindex = self.video_detect_index()
                        print("video device index : ", vindex)

                        self.cap = cv2.VideoCapture(vindex)

                        pass

                else:
                    self.rgbImg = cv2.cvtColor(cvImg, cv2.COLOR_BGR2RGB)        
                    self.grayImg = cv2.cvtColor(cvImg, cv2.COLOR_BGR2GRAY)
                    # self.rgbImg = cv2.cvtColor(self.cvImg, cv2.COLOR_BGR2RGB)        
                    # self.grayImg = cv2.cvtColor(self.cvImg, cv2.COLOR_BGR2GRAY)

                    self.rotImg = cv2.rotate(self.rgbImg, cv2.ROTATE_90_CLOCKWISE)
                    self.flipImg = self.rotImg.copy()

                    h, w, ch = self.flipImg.shape
                    bytesPerLine = ch * w

                    self.centerx = w/2
                    self.centery = h/2

                    self.rgbImg2 = self.flipImg.copy()                    
                    self.grayImg = cv2.rotate(self.grayImg, cv2.ROTATE_90_CLOCKWISE)
                    self.grayImg = cv2.GaussianBlur(self.grayImg, (7,7), 0)
                
                    edged = cv2.Canny(self.grayImg, 50, 100)
                    edged = cv2.dilate(edged, None, iterations=1)                
                    edged = cv2.erode(edged, None, iterations=1)
                    
                    # find contours in the edge map
                    contours = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    cnts = imutils.grab_contours(contours)
                    cv2.drawContours(self.rgbImg2, cnts, -1, (0, 255, 0), 4)

                    # print("get edge of image")

                    self.StartCnt += 1

                    if self.StartCnt >= 10:
                        self.bReadyCAMAdj = True

                    if self.bAdjustPos == True and self.bMoving == False and self.bReadyCAMAdj == True:
                        # print("inside image proc")
                        areas = [cv2.contourArea(c) for c in cnts]                    
                        
                        try:
                            max_index = np.argmax(areas)
                            cnt=cnts[max_index]

                            self.area = areas[max_index]
                            print("max index:" + str(max_index) + " max areas:" + str(areas[max_index]))

                            ellipse = cv2.fitEllipse(cnt)
                            angle = 90 - ellipse[2]

                            if self.bEStop is False:
                                self.readAngCnt += 1
                        except:
                            angle = 0

                        if self.area > 15000:
                            try:
                                x, y, wd, he = cv2.boundingRect(cnt)
                                cv2.rectangle(self.rgbImg2 , (x,y), (x+wd,y+he), (0,0,255), 10)
                                
                                cx = x + wd/2
                                cy = y + he/2

                                cv2.circle(self.rgbImg2 , (cx, cy), 20, (0, 0, 255), -1)

                                self.centery += self.finaloffset

                                self.diffx = self.centerx - cx
                                self.diffy = self.centery - cy  
                            except:
                                self.diffx = 0
                                self.diffy = 0

                                pass
                        else:
                            cx = 0
                            cy = 0
                            self.diffx = 0
                            self.diffy = 0

                        print("cx:", cx, " cy:", cy, "centerx:", self.centerx, "centery:", self.centery)                  
                        print("diffx:", self.diffx, " diffy:", self.diffy) 

                    if self.bAdjustPos == True and self.bMobiling == False and self.AdjStep == 1 and self.bEStop is False  and self.bReadyCAMAdj == True:
                        print("cx:", cx, " fabs(diffx):", math.fabs(self.diffx))
                        if cx <= 0 or (math.fabs(self.diffx) >= 300):
                            self.pubtwist.emit(0.03) 
                            self.movedir = 1 
                            self.estop_movdir = 0       
                            self.bMobiling = True     
                            self.AdjStep = 2           
                            self.repcnt = 0
                            self.movcnt = 100
                            print("Start moving foarward...")
                        else:
                            self.AdjStep = 3
                            self.bMobiling = True 

                    elif self.bAdjustPos == True and self.bMobiling == True and self.AdjStep == 2 and self.bEStop is False  and self.bReadyCAMAdj == True:
                        if cx <= 0:
                            self.repcnt += 1
                            if self.movedir == 0 and self.repcnt > self.movcnt:
                                self.pubtwist.emit(0.03) 
                                self.movedir = 1
                                self.repcnt = 0
                                self.movcnt = 200
                                self.estop_movdir = 0
                                print("Return forward...")
                            elif self.movedir == 1 and self.repcnt > self.movcnt:
                                self.pubtwist.emit(-0.03) 
                                self.movedir = 0
                                self.repcnt = 0
                                self.movcnt = 300
                                self.estop_movdir = 1
                                print("Return backward...")
                        elif math.fabs(self.diffx) < 100:
                            self.AdjStep = 3
                            print("repcnt :" + str(self.repcnt))
                            self.repcnt = 0
                            print("cx :" + str(cx) + " diffx: " + str(self.diffx))


                    elif self.bAdjustPos == True and self.bMobiling == True and self.AdjStep == 3 and self.bEStop is False and self.bReadyCAMAdj == True:
                        if self.diffx >= 0:
                            self.pubtwist.emit(-0.02) 
                        else:
                            self.pubtwist.emit(0.02) 

                        self.AdjStep = 4
                        self.bMobiling = True

                        self.prev_diffx = self.diffx

                        print("Start adjustment mobiling...")
                        sleep(1)

                    elif self.bAdjustPos == True and self.bMobiling == True and self.AdjStep == 4 and self.bEStop is False and self.bReadyCAMAdj == True:
                        if self.diffx <= self.movdiff and self.diffx >= -1.0*self.movdiff:    
                            self.pubtwist.emit(0.0)

                            self.AdjStep = 5
                            self.bMobiling = False
                            print("Done adjustment mobiling...")
                            sleep(1)
                        elif self.diffx < -1.0*self.movdiff and self.prev_diffx >= 0:
                            self.pubtwist.emit(0.02)
                            self.prev_diffx = self.diffx
                            print("Exceed forward x-axis moving...")
                            sleep(1)
                        elif self.diffx > self.movdiff and self.prev_diffx <= 0:
                            self.pubtwist.emit(-0.02)
                            self.prev_diffx = self.diffx
                            print("Exceed backward x-axis moving...")
                            sleep(1)

                    if self.bAdjustPos == True and self.bMoving == False and self.AdjStep == 5 and self.bReadyCAMAdj == True: 
                        zbdsr_move = ZBDSRMove()
                        zbdsr_move.xpos = prev_xpos

                        if math.fabs(self.diffy) > self.diffconsty:
                            dval = self.diffy*self.mov_gainz
                            zbdsr_move.zpos = prev_zpos + dval

                            zbdsr_move.rxpos = prev_rxpos
                            zbdsr_move.rypos = prev_rypos
                            zbdsr_move.rzpos = prev_rzpos
                            zbdsr_move.setstatus = ZBDSRMove.START

                            self.pub_movpos.publish(zbdsr_move)

                            self.trajStatus = ZBDSRTrajStatus.IDLING
                            prev_ypos = zbdsr_move.ypos
                            prev_zpos = zbdsr_move.zpos
                            prev_rzpos = zbdsr_move.rzpos
                            
                            self.bMoving = True

                            mstr = "DSR Robot Moving Pos(" + str(self.diffx) + ", " + str(self.diffy) + ")"
                            print(mstr)
                            self.print_mess.emit(mstr)

                            print("dx : " + str(self.diffx*self.mov_gainy) + "   dy : " +str(self.diffy*self.mov_gainz))

                        elif math.fabs(self.diffy) <= self.diffconsty:
                            self.AdjStep = 6
                            self.angleAdjCnt = 0
                            self.readAngCnt = 0
                            print("done adjustment x-axis")

                            sval = Bool()
                            sval.data = True
                            self.pub_getdsrpos.publish(sval)

                            self.bGetPosOk = False
                            print("get dsr position...")

                    if self.AdjStep == 6 and self.bMoving == False and self.bGetPosOk == True and self.bReadyCAMAdj == True:
                        self.bGetPosOk = False

                        print("get pos :" + str(self.getPosVal[0]) + ", " + str(self.getPosVal[1]) + ", " + str(self.getPosVal[2]) + ", " + str(self.getPosVal[3]) + ", " + str(self.getPosVal[4]) + ", " + str(self.getPosVal[5]))

                        zbdsr_move.xpos = self.getPosVal[0] + self.fmoveval1
                        zbdsr_move.ypos = self.getPosVal[1]
                        zbdsr_move.zpos = self.getPosVal[2]
                        zbdsr_move.rxpos = self.getPosVal[3]
                        zbdsr_move.rypos = self.getPosVal[4]
                        zbdsr_move.rzpos = self.getPosVal[5]

                        zbdsr_move.setstatus = ZBDSRMove.START

                        self.pub_movpos.publish(zbdsr_move)

                        self.trajStatus = ZBDSRTrajStatus.IDLING

                        print("Send X-axis moving command...")

                        self.bMoving = True

                        prev_xpos = zbdsr_move.xpos
                        prev_ypos = zbdsr_move.ypos
                        prev_zpos = zbdsr_move.zpos

                        prev_rxpos = zbdsr_move.rxpos
                        prev_rypos = zbdsr_move.rypos
                        prev_rzpos = zbdsr_move.rzpos

                    # if self.bAdjustPos == True and self.bMoving == False and self.AdjStep == 5 or self.AdjStep == 8) and self.bEStop is False:

                    #     zbdsr_move = ZBDSRMove()
                    #     zbdsr_move.xpos = prev_xpos

                    #     if self.AdjStep == 5 or self.AdjStep == 8:
                    #         if math.fabs(self.diffx) > self.diffconstx:
                    #             zbdsr_move.ypos = prev_ypos + self.diffx*self.mov_gainy
                    #         else:
                    #             zbdsr_move.ypos = prev_ypos
                    #     else:
                    #         zbdsr_move.ypos = prev_ypos

                    #     if self.AdjStep == 5 or self.AdjStep == 8:
                    #         if math.fabs(self.diffy) > self.diffconsty:
                    #             dval = self.diffy*self.mov_gainz                                
                    #             zbdsr_move.zpos = prev_zpos + dval
                    #         else:
                    #             zbdsr_move.zpos = prev_zpos
                    #     else:
                    #         zbdsr_move.zpos = prev_zpos

                    #     if math.fabs(self.diffx) <= self.diffconstx and math.fabs(self.diffy) <= self.diffconsty:
                    #         if self.AdjStep == 5:
                    #             self.AdjStep = 6
                    #             self.angleAdjCnt = 0
                    #             self.readAngCnt = 0
                    #             print("done first-time adjust x-axis")
                    #         elif self.AdjStep == 8:
                    #             self.AdjStep = 9
                    #             print("done second-time adjust x-axis")

                    #             sval = Bool()
                    #             sval.data = True
                    #             self.pub_getdsrpos.publish(sval)

                    #             self.bGetPosOk = False
                    #             print("[2]get dsr position...")
                    #     else:
                    #         mstr = "Start DSR Robot Moving Pos(" + str(self.diffx) + ", " + str(self.diffy) + ")"
                    #         print(mstr)
                    #         self.print_mess.emit(mstr)

                    #         print("dx : " + str(self.diffx*self.mov_gainy) + "   dy : " +str(self.diffy*self.mov_gainz))

                    #     if self.AdjStep == 5 or self.AdjStep == 8:
                    #         zbdsr_move.rxpos = prev_rxpos
                    #         zbdsr_move.rypos = prev_rypos
                    #         zbdsr_move.rzpos = prev_rzpos
                    #         zbdsr_move.setstatus = ZBDSRMove.START

                    #         self.pub_movpos.publish(zbdsr_move)

                    #         self.trajStatus = ZBDSRTrajStatus.IDLING

                    #         prev_ypos = zbdsr_move.ypos
                    #         prev_zpos = zbdsr_move.zpos
                    #         prev_rzpos = zbdsr_move.rzpos

                    #         self.bMoving = True

                    # if self.AdjStep == 7 and self.bMoving == False and self.bGetPosOk == True and self.bEStop is False:
                    #     self.bGetPosOk = False

                    #     print("[step7] get pos :" + str(self.getPosVal[0]) + ", " + str(self.getPosVal[1]) + ", " + str(self.getPosVal[2]) + ", " + str(self.getPosVal[3]) + ", " + str(self.getPosVal[4]) + ", " + str(self.getPosVal[5]))

                    #     zbdsr_move.xpos = self.getPosVal[0]
                    #     zbdsr_move.ypos = self.getPosVal[1]
                    #     zbdsr_move.zpos = self.getPosVal[2]
                    #     zbdsr_move.rxpos = self.getPosVal[3]
                    #     zbdsr_move.rypos = self.getPosVal[4]
                    #     zbdsr_move.rzpos = self.getPosVal[5]
                    #     self.AdjStep = 8
                    #     self.bMoving = False

                    # if self.AdjStep == 9 and self.bMoving == False and self.bGetPosOk == True and self.bEStop is False:
                    #     self.bGetPosOk = False

                    #     print("[step9] get pos :" + str(self.getPosVal[0]) + ", " + str(self.getPosVal[1]) + ", " + str(self.getPosVal[2]) + ", " + str(self.getPosVal[3]) + ", " + str(self.getPosVal[4]) + ", " + str(self.getPosVal[5]))

                    #     zbdsr_move.xpos = self.getPosVal[0] + self.fmoveval1
                    #     zbdsr_move.ypos = self.getPosVal[1]
                    #     zbdsr_move.zpos = self.getPosVal[2]
                    #     zbdsr_move.rxpos = self.getPosVal[3]
                    #     zbdsr_move.rypos = self.getPosVal[4]
                    #     zbdsr_move.rzpos = self.getPosVal[5]

                    #     zbdsr_move.setstatus = ZBDSRMove.START

                    #     # prev_xpos = zbdsr_move.xpos
                    #     # prev_ypos = zbdsr_move.ypos
                    #     # prev_zpos = zbdsr_move.zpos
                    #     # prev_rxpos = zbdsr_move.rxpos
                    #     # prev_rypos = zbdsr_move.rypos
                    #     # prev_rzpos = zbdsr_move.rzpos

                    #     self.pub_movpos.publish(zbdsr_move)

                    #     self.trajStatus = ZBDSRTrajStatus.IDLING

                    #     print("Send X-axis moving command...")

                    #     self.bMoving = True

                    # if self.AdjStep == 6 and self.bMoving == False:
                    #     print("readAngCnt:" + str(self.readAngCnt))
                    #     if self.readAngCnt >= 3:
                    #         if math.fabs(angle) >= self.diffangle:
                    #             self.readAngCnt = 0
                    #             if math.fabs(angle) > self.initangle:
                    #                 if angle > 0.0:
                    #                     angle = self.initangle
                    #                 else:
                    #                     angle = -1.0*self.initangle

                    #             zbdsr_move.rzpos = prev_rzpos  - angle/self.divangle

                    #             zbdsr_move.rxpos = prev_rxpos
                    #             zbdsr_move.rypos = prev_rypos

                    #             if self.angleAdjCnt >= 10:
                    #                 zbdsr_move.rzpos = prev_rzpos
                    #                 # self.AdjStep = 7
                    #                 self.angleAdjCnt = 0
                    #                 self.divangle *= 1.5
                    #                 print("time out angle adj....")
                    #             else:
                    #                 self.angleAdjCnt += 1

                    #                 zbdsr_move.setstatus = ZBDSRMove.START
                    #                 self.pub_movpos.publish(zbdsr_move)
                    #                 self.trajStatus = ZBDSRTrajStatus.IDLING

                    #                 print("math.fabs(angle) : " + str(math.fabs(angle)))
                    #                 print("rotate angle:" + str(angle/self.divangle) + " prev rzpos:" +str(prev_rzpos))

                    #                 print("Send RZ-axis moving command...")

                    #                 self.bMoving = True

                    #         else:
                    #             zbdsr_move.rzpos = prev_rzpos
                    #             self.AdjStep = 7
                    #             self.readAngCnt = 0

                    #             print("angle:" + str(angle))
                    #             print("done adjust rz-axis")

                    #             sval = Bool()
                    #             sval.data = True
                    #             self.pub_getdsrpos.publish(sval)

                    #             self.bGetPosOk = False
                    #             print("[1] get dsr position...")
                    #     else:
                    #         zbdsr_move.rzpos = prev_rzpos

                    if self.bAdjustPos == True and self.bMoving == True and self.trajStatus == ZBDSRTrajStatus.COMPLETED  and self.bEStop is False and self.bReadyCAMAdj == True:
                        sleep(0.5)

                        self.trajStatus = ZBDSRTrajStatus.IDLING
                        self.bMoving = False

                        if self.AdjStep == 6:
                            self.AdjStep = 7
                            # self.clearscene.emit(2)

                        mstr = "End Moving Pos of DSR Robot..."
                        print(mstr)
                        self.print_mess.emit(mstr)

                    if self.bAdjustPos == True:
                        cv2.putText(self.rgbImg2, 'cx,cy=({0}, {1})'.format(self.centerx, self.centery), (50, 420+300), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 255, 0), 5)
                        cv2.putText(self.rgbImg2, 'x,y=({0}, {1})'.format(cx, cy), (50, 420+450), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 0, 0), 5)
                        cv2.putText(self.rgbImg2, 'dx,dy=({0}, {1})'.format(self.diffx, self.diffy), (50, 420+600), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 0, 255), 5)
                        cv2.putText(self.rgbImg2, 'angle= {0}'.format(angle), (50, 420+750), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 255, 255), 5)

                        h2, w2, ch2 = self.rgbImg2.shape
                        bytesPerLine2 = ch2 * w2

                        qt_image = QImage(self.rgbImg2.data, w2, h2, bytesPerLine2, QImage.Format_RGB888)
                        p = qt_image.scaled(151, 201, Qt.KeepAspectRatio)
                        self.VideoSignal3.emit(p)

                    self.qtImg = QImage(self.flipImg.data, w, h, bytesPerLine, QImage.Format_RGB888)
                    self.scaledImg = self.qtImg.scaled(151, 201, Qt.KeepAspectRatio)
                    self.VideoSignal2.emit(self.scaledImg)

                    # self.bGetImgOK = False
                    self.bImgProcOK = False
                    
            if self.AdjStep == 7 and self.bReadyCAMAdj == True:
                break

            sleep(0.1)

        self.clearscene.emit(2)

        self.endadj.emit()

        self.bAdjustPos = False
        self.bMoving = False
        self.AdjStep = 0
        self.bGetPosOk = False
        self.bMobiling = False

        # self.setmovbtn.emit(False)

        print("Exit runCAMAdjProcess...")


    def pubRobotcontrolStatus(self, stat):
        rcs = RobotControlStatus()
        rcs.status = stat
        
        if stat == RobotControlStatus.ERROR:
            rcs.status_description = "Robot OP Status : Error"
        elif stat == RobotControlStatus.IDLING:
            rcs.status_description = "Robot OP Status : Idling"
        elif stat == RobotControlStatus.NAVI:
            rcs.status_description = "Robot OP Status : Waypoint navigation"
        elif stat == RobotControlStatus.DSR:
            rcs.status_description = "Robot OP Status : Moving DSR robot"
        elif stat == RobotControlStatus.UV:
            rcs.status_description = "Robot OP Status : Using UV Lamp"
        elif stat == RobotControlStatus.SPARY:
            rcs.status_description = "Robot OP Status : Using Spary pump"
        elif stat == RobotControlStatus.UVSPARY:
            rcs.status_description = "Robot OP Status : Using Spary pump and UV Lamp"
        elif stat == RobotControlStatus.CHARGING:
            rcs.status_description = "Robot OP Status : Charging in progress"
            
        self.rcspub.publish(rcs) 
            
    @pyqtSlot(int, str)
    def setNaviStat(self, status, stat_descript):
        self.naviStatus = status

        if(self.prevnaviStatus != self.naviStatus):
            mstr = stat_descript
            print(mstr)

            print("naviStatus : "+str(self.naviStatus))

            self.prevnaviStatus = self.naviStatus
              
    @pyqtSlot(str)
    def setCSStatus(self, stat):
        if self.bCSParking == True:
            self.csStatus = stat

            mstr = "Charging Station Status : " + self.csStatus
            
    @pyqtSlot(int, int)
    def setDSRTraj(self, curstatus, trajnum):
        stat_str = "none"
        
        if curstatus == ZBDSRTrajStatus.IDLING:
            stat_str = "Idling"
            self.trajStatus = ZBDSRTrajStatus.IDLING
        elif curstatus == ZBDSRTrajStatus.RUNNING:
            stat_str = "Running"
            self.trajStatus = ZBDSRTrajStatus.RUNNING
        elif curstatus == ZBDSRTrajStatus.PAUSED:
            stat_str = "Paused"
            self.trajStatus = ZBDSRTrajStatus.PAUSED
        elif curstatus == ZBDSRTrajStatus.COMPLETED:
            stat_str = "Completed"
            self.trajStatus = ZBDSRTrajStatus.COMPLETED
        elif curstatus == ZBDSRTrajStatus.CANCELLED:
            stat_str = "Canceled"
            self.trajStatus = ZBDSRTrajStatus.CANCELLED
        elif curstatus == ZBDSRTrajStatus.START:
            stat_str = "Start"
            self.trajStatus = ZBDSRTrajStatus.START
        elif curstatus == ZBDSRTrajStatus.STOP:
            stat_str = "Stop"
            self.trajStatus = ZBDSRTrajStatus.STOP
        elif curstatus == ZBDSRTrajStatus.ESTOP:
            stat_str = "EStop"
            self.trajStatus = ZBDSRTrajStatus.ESTOP
        elif curstatus == ZBDSRTrajStatus.RESTART:
            stat_str = "Restart"
            self.trajStatus = ZBDSRTrajStatus.RESTART
            
        mstr = "[HDIP]TrajNum(" + str(trajnum) + "), Status:" + stat_str
        print(mstr)
        
    def StopWaypoint(self):
        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.wpName
        self.ncpub.publish(nc)
        mstr = "[WP] Stop way point : " + self.wpName
        print(mstr)        
        self.navi_runstatstr.emit(mstr)
        
        self.pubRobotcontrolStatus(RobotControlStatus.IDLING)
        
    def gotoWayPoint(self, wpname):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = wpname
        self.ncpub.publish(nc)
        
        self.wpName = wpname
        
        mstr = "[WP] Start hollow disinfection way point : " + nc.goal_name
        print(mstr)
        
        self.pubRobotcontrolStatus(RobotControlStatus.NAVI)

    def RunDSRTraj(self, trajnum):
        zbdsr_traj = ZBDSRSetTraj()
        self.cur_trajnum = trajnum
        zbdsr_traj.trajnum = trajnum
        zbdsr_traj.setstatus = ZBDSRSetTraj.START
        self.dsrpub.publish(zbdsr_traj)
        
        self.pubRobotcontrolStatus(RobotControlStatus.DSR)
        
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
        self.navi_runstatstr.emit(mstr)
        
        self.pubRobotcontrolStatus(RobotControlStatus.IDLING)

    def CheckEStop(self):

        if self.bEStop == True:

            while True:

                if self.bEStop == False:
                    break

                sleep(0.2)

    def CheckAborted(self, wp):
        if self.naviStatus == NavigationControlStatus.ABORTED:
            self.navi_runstatstr.emit("Navi. Aborted")

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
        global HDSch_List
        global bUseCAMAdj

        runcnt = 0

        bErrFlag = False

        self.setlaser.emit(True)

        self.setuvblamp.emit(True)
        
        proc_num = 0
        
        for slist in HDSch_List:

            if self.bRunFlag == False:
                print("Stopping Navi Process... ")
                break

            proc_name = slist["name"] 
            proc_type = slist["type"]
            proc_sp = slist["start_point"]
            proc_ap = slist["adjust_point"]
            proc_ep = slist["end_point"]

            if bUseCAMAdj == True:
                proc_ftnum = slist["dsrftnum"]
            else:
                proc_ftnum = slist["dsrnftnum"]

            proc_btnum = slist["dsrbtnum"]

            # print("----------------------")
            # print("name : " + proc_name)
            # print("type : " + proc_type)
            # print("start point : " + proc_sp)
            # print("adjust point : " + proc_ap)
            # print("dsrftnum : " + str(proc_ftnum))
            # print("dsrbtnum : " + str(proc_btnum))
            # print("end point : " + proc_ep)
            # print("----------------------")   

            print("[1]type : " + proc_type)
            
            if proc_type == "normal":
                # =================================
                # Hallow Disinfection Process #1
                # =================================
                
                if self.bRunFlag == True:   
                # if (self.HDIRun[proc_num] == '0') and self.bRunFlag == True:   
                    self.naviStatus = NavigationControlStatus.IDLING

                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                    print("[2]start point : " + proc_sp)
                    self.gotoWayPoint(proc_sp)
                    # self.gotoWayPoint("device1")

                    sleep(5)

                    runcnt = 0
                    
                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            
                            continue

                        self.CheckAborted(proc_sp)
                        # self.CheckAborted("device1")
                        
                        if self.bEStop == False and self.bprevEStop == True:
                            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                            
                            sleep(5)
                            
                            self.bprevEStop = False

                            self.gotoWayPoint(proc_sp)
                            # self.gotoWayPoint("device1")

                            self.navi_runstatstr.emit("Restart " + proc_sp)

                            print("Restart " + proc_sp)

                            self.naviStatus = NavigationControlStatus.IDLING

                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.navi_runstatstr.emit("Stop " + proc_sp + "HDI process...")

                            self.setrunflag.emit(True)

                            sleep(1)

                            self.stop()

                            bErrFlag = True

                            break

                        if self.bRunPauseFlag == True:
                            self.navi_runstatstr.emit("Pause " + proc_sp + "HDI process...")
                            # self.navi_runstat.emit("Pause device1 HDI process...")

                            while self.bRunPauseFlag:
                                if self.bRunPauseFlag is False:
                                    self.navi_runstatstr.emit("Restart " + proc_sp + "HDI process...")
                                    # self.navi_runstat.emit("Restart device1 HDI process...")
                                    break
                            
                                sleep(0.1)            
                
                        if self.naviStatus == NavigationControlStatus.COMPLETED:
                            #self.HDIRun[2] = 1

                            print("Arrived at " + proc_sp + "...")
                            # print("Arrived at device1...")
                            break

                        if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                            runcnt = runcnt + 1

                            if runcnt <= self.RepCnt:
                                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                                self.gotoWayPoint(proc_sp)
                                # self.gotoWayPoint("device1")

                                self.navi_runstatstr.emit("Restart " + proc_sp)
                                # self.navi_runstat.emit("Restart device1")

                                print("Restart " + proc_sp + " : " + str(runcnt))
                                # print("Restart Device1 : " + str(runcnt))

                                sleep(2)

                            else:
                                runcnt = 0

                                self.navi_runstatstr.emit("Error(WARNNRGTO) " + proc_sp)
                                # self.navi_runstat.emit("Error(WARNNRGTO) device1")

                            break

                        if self.naviStatus == NavigationControlStatus.ERRGTGF:
                            self.navi_runstatstr.emit("Error(ERRGTGF) " + proc_sp)
                            # self.navi_runstat.emit("Error(ERRGTGF) device1")

                            #self.HDIRun[2] = 0

                        sleep(0.1)


                    if self.bRunFlag == True:
                        sleep(1)

                # =================================
                # Hallow Disinfection Process #2
                # =================================

                if self.bRunFlag == True:     
                # if self.HDIRun[proc_num] == '0' and self.bRunFlag == True:     
                    self.naviStatus = NavigationControlStatus.IDLING
        
                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                    print("[3]adjust point : " + proc_ap)
                    self.gotoWayPoint(proc_ap)
                    # self.gotoWayPoint("device1_1")

                    sleep(5)

                    runcnt = 0

                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue

                        self.CheckAborted(proc_ap)
                        # self.CheckAborted("device1_1")

                        if self.bEStop == False and self.bprevEStop == True:
                            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                            
                            sleep(5)
                            
                            self.bprevEStop = False

                            self.gotoWayPoint(proc_ap)
                            # self.gotoWayPoint("device1_1")

                            self.navi_runstatstr.emit("Restart " + proc_ap)
                            # self.navi_runstat.emit("Restart Device1_1")

                            print("Restart " + proc_ap)
                            # print("Restart Device1_1")

                            self.naviStatus = NavigationControlStatus.IDLING

                            
                        if self.bRunFlag == False:
                            self.StopWaypoint()
                            
                            self.navi_runstatstr.emit("Stop " + proc_ap + "HDI process...")
                            # self.navi_runstat.emit("Stop device1_1 HDI process...")

                            self.setrunflag.emit(True)

                            sleep(1)
                            
                            self.stop()

                            bErrFlag = True

                            break
                            
                        if self.bRunPauseFlag == True:
                            self.navi_runstatstr.emit("Pause " + proc_ap + "HDI process...")
                            # self.navi_runstat.emit("Pause device1_1 HDI process...")
                            while self.bRunPauseFlag:
                                if self.bRunPauseFlag is False:
                                    self.navi_runstatstr.emit("Restart " + proc_ap + "HDI process...")
                                    # self.navi_runstat.emit("Restart device1_1 HDI process...")
                                    break
                            
                                sleep(0.1)
                    
                        if self.naviStatus == NavigationControlStatus.COMPLETED:
                            # self.HDIRun[0] = 1

                            print("Arrived at " + proc_ap + "...")
                            # print("Arrived at device1_1...")

                            bErrFlag = False
                            break

                        if self.naviStatus == NavigationControlStatus.WARNNRGTO or self.naviStatus == NavigationControlStatus.ERRGTGF:
                            runcnt = runcnt + 1

                            if runcnt <= self.RepCnt:
                                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                                self.gotoWayPoint(proc_ap)
                                # self.gotoWayPoint("device1_1")

                                self.navi_runstatstr.emit("Restart " + proc_ap)
                                # self.navi_runstat.emit("Restart device1_1")

                                print("Restart " + proc_ap + " : " + str(runcnt))
                                # print("Restart device1_1 : " + str(runcnt))

                                self.naviStatus = NavigationControlStatus.IDLING

                                sleep(2)

                            else:
                                # self.HDIRun[0] = 0

                                runcnt = 0

                                self.navi_runstatstr.emit("Error(WARNNRGTO) " + proc_ap)
                                # self.navi_runstat.emit("Error(WARNNRGTO) device1_1")

                                print("Error(WARNNRGTO) device1_1")

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

                        self.CheckEStop()

                        self.trajStatus = ZBDSRTrajStatus.IDLING

                        print("[4]dsrftnum : " + str(proc_ftnum))
                        
                        self.RunDSRTraj(proc_ftnum)
                        # self.RunDSRTraj(9)

                        print("[" + proc_ap + "] Send topic [Run Trajectory " + str(proc_ftnum) + "]")
                        # print("[device1_1] Send topic [Run Trajectory 9]")

                        while True:
                            if self.bEStop == True:
                                sleep(0.1)
                                continue

                            if self.bEStop == False and self.bprevEStop == True:
                                self.trajStatus = ZBDSRTrajStatus.RESTART
                                print("====> delay 6 sec...")
                                sleep(6)
                                if self.bEStop == False:                                   
                                    self.bprevEStop = False
                            
                            if self.bRunFlag == False:
                                self.stopTraj()

                                self.dsrtraj_runstatstr.emit("Stop moving HDI traj. #" + str(proc_ftnum) + "...")
                                # self.dsrtraj_runstat.emit("Stop moving HDI traj. #9...")
                                    
                                sleep(1)
                                    
                                self.stop()

                                break
                                                                
                            if self.trajStatus == ZBDSRTrajStatus.COMPLETED and self.bEStop == False:
                                # 2022. 09. 03
                                self.CheckEStop()

                                # 2022. 08. 08
                                if bUseCAMAdj == True:
                                    # self.SetCAMAdapt(True)
                                    # sleep(1)
                                    self.SetAdjustFlag(True)

                                    self.runCAMAdjProcess()
                                                                       
                                    print(">>> [ZBDSRTraj] => End runCAMAdjProcess...")

                                break

                            sleep(0.1)

                        if self.bRunFlag == True:
                            self.dsrtraj_runstatstr.emit("Complete moving HDI Trjaectory #" + str(proc_ftnum) + "...")        
                            # self.dsrtraj_runstat.emit("Complete moving HDI Trjaectory #9...")        

                            self.CheckEStop()      
        
                            self.pubRobotcontrolStatus(RobotControlStatus.UV)
                            self.setuvlamp.emit(True)

                            print("Waiting during disinfection...")

                            self.CheckEStop()

                            sleep(DTRunTime)

                            self.CheckEStop()

                            self.setuvlamp.emit(False)
                            self.pubRobotcontrolStatus(RobotControlStatus.IDLING)
                                    
                            sleep(3)

                            self.CheckEStop()

                            zbdsr_traj = ZBDSRSetTraj()            
                            zbdsr_traj.trajnum = proc_ftnum
                            # zbdsr_traj.trajnum = 9
                            zbdsr_traj.setstatus = ZBDSRSetTraj.STOP
                            self.dsrpub.publish(zbdsr_traj)

                            sleep(2)

                            self.CheckEStop()

                            print("[5]dsrbtnum : " + str(proc_btnum))

                            self.RunDSRTraj(proc_btnum)
                            # self.RunDSRTraj(10)

                            print("[" + proc_ap + "] Send topic [Run Trajectory " + str(proc_btnum) + "]")
                            # print("[device1_1] Send topic [Run Trajectory 10]")

                            self.trajStatus = ZBDSRTrajStatus.IDLING

                            while True:
                                if self.bEStop == True:
                                    sleep(0.1)
                                    continue

                                if self.bEStop == False and self.bprevEStop == True:
                                    print("====> delay 6 sec...")
                                    sleep(6)

                                    if self.bEStop == False:                                   
                                        self.bprevEStop = False
                                    
                                if self.bRunFlag == False:
                                    self.stopTraj()

                                    self.dsrtraj_runstatstr.emit("Stop moving HDI traj. #" + str(proc_btnum) + "...")
                                    # self.dsrtraj_runstat.emit("Stop moving HDI traj. #10...")
                                    
                                    sleep(1)
                                    
                                    self.stop()

                                    break
                            
                                if self.trajStatus == ZBDSRTrajStatus.COMPLETED:
                                    self.CheckEStop()

                                    print("return traj ending...")
                                    break

                                sleep(0.1)

                            if self.bRunFlag == True:
                                print("Complete moving HDI Trjaectory #" + str(proc_btnum) + "...")
                                # print("Complete moving HDI Trjaectory #10...")

                                self.CheckEStop()

                                self.setfwf.emit(False)
                                print("Turn off Front WP")

                                sleep(0.5)

                                self.CheckEStop()

                                self.setbwf.emit(False)
                                print("Turn off Rear WP")

                                sleep(5)

                                print("Complete " + proc_name + "device disinfection...")

                    else:
                        print("Skip " + proc_name + "device disinfection...")
                        # print("Skip first device disinfection...")

                bErrFlag = False

                # ==================================
                # Hallow Disinfection Process #3
                # ==================================
                
                if self.bRunFlag == True:   
                    self.naviStatus = NavigationControlStatus.IDLING

                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                    print("[5]end point : " + proc_ep)
                    self.gotoWayPoint(proc_ep)
                    # self.gotoWayPoint("device1_2")

                    sleep(5)

                    runcnt = 0
                    
                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue

                        self.CheckAborted(proc_ep)
                        # self.CheckAborted("device1_2")
                        
                        if self.bEStop == False and self.bprevEStop == True:
                            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                            
                            sleep(5)
                            
                            self.bprevEStop = False

                            self.gotoWayPoint(proc_ep)
                            # self.gotoWayPoint("device1_2")

                            self.navi_runstatstr.emit("Restart " + proc_ep)
                            # self.navi_runstat.emit("Restart Device1")

                            print("Restart " + proc_ep)
                            # print("Restart Device1")

                            self.naviStatus = NavigationControlStatus.IDLING
                        

                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.navi_runstatstr.emit("Stop ret " + proc_ep + " HDI process...")
                            # self.navi_runstat.emit("Stop ret device1_2 HDI process...")

                            self.setrunflag.emit(True)

                            sleep(1)

                            self.stop()

                            bErrFlag = True

                            break

                        if self.bRunPauseFlag == True:
                            self.navi_runstatstr.emit("Pause ret " + proc_ep + " HDI process...")
                            # self.navi_runstat.emit("Pause ret device1_2 HDI process...")
                            while self.bRunPauseFlag:
                                if self.bRunPauseFlag is False:
                                    self.navi_runstatstr.emit("Restart ret " + proc_ep + " HDI process...")
                                    # self.navi_runstat.emit("Restart ret device1_2 HDI process...")
                                    break
                            
                                sleep(0.1)            
                
                        if self.naviStatus == NavigationControlStatus.COMPLETED:
                            print("Arrived at ret " + proc_ep + "...")
                            # print("Arrived at ret device1_2...")
                            break

                        if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                            runcnt = runcnt + 1

                            if runcnt <= self.RepCnt:
                                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                                self.gotoWayPoint(proc_ep)
                                # self.gotoWayPoint("device1_2")

                                self.navi_runstatstr.emit("Restart ret " + proc_ep)
                                # self.navi_runstat.emit("Restart ret device1_2")

                                print("Restart ret " + proc_ep + " : " + str(runcnt))
                                # print("Restart ret Device1 : " + str(runcnt))

                                sleep(2)

                            else:
                                runcnt = 0

                                self.navi_runstatstr.emit("Error(WARNNRGTO) ret " + proc_ep)
                                # self.navi_runstat.emit("Error(WARNNRGTO) ret device1_2")

                            break

                        if self.naviStatus == NavigationControlStatus.ERRGTGF:
                            self.navi_runstatstr.emit("Error(ERRGTGF) ret " + proc_ep)
                            # self.navi_runstat.emit("Error(ERRGTGF) ret device1_2")

                        sleep(0.1)


                    if self.bRunFlag == True:
                        sleep(2)
                        
                proc_num += 1
            
            elif proc_type == "waypoint":
                # ========================================
                # Hallow Disinfection Process waypoint
                # ========================================
                
                if self.bRunFlag == True:   
                # if (self.HDIRun[2] == '0' or self.HDIRun[3] == '0') and self.bRunFlag == True:   
                    self.naviStatus = NavigationControlStatus.IDLING

                    os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                    self.gotoWayPoint(proc_sp)
                    # self.gotoWayPoint("lobbypos4")

                    sleep(5)

                    runcnt = 0
                    
                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue

                        self.CheckAborted(proc_sp)
                        # self.CheckAborted("lobbypos4")
                        
                        if self.bEStop == False and self.bprevEStop == True:
                            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                            
                            sleep(5)
                            
                            self.bprevEStop = False

                            self.gotoWayPoint(proc_sp)
                            # self.gotoWayPoint("lobbypos4")

                            self.navi_runstatstr.emit("Restart " + proc_sp)
                            # self.navi_runstat.emit("Restart Lobbypos4")

                            print("Restart " + proc_sp)
                            # print("Restart Lobbypos4")

                            self.naviStatus = NavigationControlStatus.IDLING
                            

                        if self.bRunFlag == False:
                            self.stopTraj()

                            self.navi_runstatstr.emit("Stop " + proc_sp + " HDI process...")
                            # self.navi_runstat.emit("Stop lobbypos4 HDI process...")

                            self.setrunflag.emit(True)

                            sleep(1)

                            self.stop()

                            bErrFlag = True

                            break

                        if self.bRunPauseFlag == True:
                            self.navi_runstatstr.emit("Pause " + proc_sp +" HDI process...")
                            # self.navi_runstat.emit("Pause lobbypos4 HDI process...")
                            while self.bRunPauseFlag:
                                if self.bRunPauseFlag is False:
                                    self.navi_runstatstr.emit("Restart " + proc_sp + " HDI process...")
                                    # self.navi_runstat.emit("Restart lobbypos4 HDI process...")
                                    break
                            
                                sleep(0.1)            
                
                        if self.naviStatus == NavigationControlStatus.COMPLETED:
                            #self.HDIRun[4] = 1

                            print("Arrived at " + proc_sp + "...")
                            # print("Arrived at lobbypos4...")
                            break

                        if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                            runcnt = runcnt + 1

                            if runcnt <= self.RepCnt:
                                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                                self.gotoWayPoint(proc_sp)
                                # self.gotoWayPoint("lobbypos4")

                                self.navi_runstatstr.emit("Restart " + proc_sp)
                                # self.navi_runstat.emit("Restart lobbypos4")

                                print("Restart " + proc_sp + " : " + str(runcnt))
                                # print("Restart lobbypos4 : " + str(runcnt))

                            else:
                                runcnt = 0

                                self.navi_runstatstr.emit("Error(WARNNRGTO) " + proc_sp)
                                # self.navi_runstat.emit("Error(WARNNRGTO) lobbypos4")

                            break

                        if self.naviStatus == NavigationControlStatus.ERRGTGF:
                            self.navi_runstatstr.emit("Error(ERRGTGF) " + proc_sp)
                            # self.navi_runstat.emit("Error(ERRGTGF) lobbypos4")

                        sleep(0.1)


                    if self.bRunFlag == True:
                        sleep(2)
                        
                proc_num += 1
                
            elif proc_type == "parking":
                
                # =======================================
                # Hallow Disinfection Process : Parking
                # =======================================

                bStartCharging = False
                
                self.naviStatus = NavigationControlStatus.IDLING

                os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                # self.gotoWayPoint(proc_sp)
                # # self.gotoWayPoint("chargingpos")

                # sleep(5)

                # runcnt = 0
                
                # while True:      
                #     if self.bEStop == True:
                #         sleep(0.1)
                #         continue

                #     self.CheckAborted(proc_sp)
                #     # self.CheckAborted("chargingpos")
                    
                #     if self.bEStop == False and self.bprevEStop == True:
                #             os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                            
                #             sleep(5)
                            
                #             self.bprevEStop = False

                #             self.gotoWayPoint(proc_sp)
                #             # self.gotoWayPoint("chargingpos")

                #             self.navi_runstatstr.emit("Restart " + proc_sp)
                #             # self.navi_runstat.emit("Restart Chargingpos")

                #             print("Restart " + proc_sp)
                #             # print("Restart Chargingpos")

                #             self.naviStatus = NavigationControlStatus.IDLING

                #     if self.bRunFlag == False:
                #         self.StopWaypoint()

                #         self.navi_runstatstr.emit("Stop " + proc_sp + " HDI process...")
                #         # self.navi_runstat.emit("Stop chargingpos HDI process...")

                #         self.setrunflag.emit(True)
                        
                #         sleep(1)
                        
                #         self.stop()

                #         break
                        
                #     if self.bRunPauseFlag == True:
                #         self.navi_runstatstr.emit("Pause " + proc_sp + " HDI process...")
                #         # self.navi_runstat.emit("Pause chargingpos HDI process...")
                #         while self.bRunPauseFlag:
                #             if self.bRunPauseFlag is False:
                #                 self.navi_runstatstr.emit("Restart " + proc_sp + " HDI process...")
                #                 # self.navi_runstat.emit("Restart chargingpos HDI process...")
                #                 break
                        
                #             sleep(0.1)
                
                #     if self.naviStatus == NavigationControlStatus.COMPLETED:
                        
                #         #self.HDIRun[6] = 1

                #         print("Arrived at " + proc_sp + "...")
                #         # print("Arrived at chargingpos...")
                #         break

                #     if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                #         runcnt = runcnt + 1

                #         if runcnt < self.RepCnt:
                #             os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                #             self.gotoWayPoint(proc_sp)
                #             # self.gotoWayPoint("chargingpos")

                #             self.navi_runstatstr.emit("Restart " + proc_sp)
                #             # self.navi_runstat.emit("Restart chargingpos")

                #             print("Restart " + proc_sp + " : " + str(runcnt))
                #             # print("Restart chargingpos : " + str(runcnt))

                #             sleep(2)

                #         else:

                #             runcnt = 0

                #             self.navi_runstatstr.emit("Error(WARNNRGTO) " + proc_sp)
                #             # self.navi_runstat.emit("Error(WARNNRGTO) chargingpos")

                #         break

                #     sleep(0.1)

                if self.bRunFlag == True:

                    # print("Arrived at charging station position...")  
                    
                    # # 2022.03.02
                    # self.setlaser.emit(False)

                    # # 2022. 07. 25
                    # self.setuvblamp.emit(False)
                    # # self.setuvlamp.emit(False)

                    client = actionlib.SimpleActionClient('charging_act', ChargingAction)
                    client.wait_for_server()

                    goal = ChargingGoal()
                    client.send_goal(goal)

                    self.bCSParking = True

                    print("Parking charging station...")

                    while True:
                        if self.bEStop == True:
                            sleep(0.1)
                            continue

                        if self.bRunFlag == False:
                            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
                            os.system(cmdstr)

                            self.navi_runstatstr.emit("Stop auto charging process...")
                            
                            sleep(1)
                            
                            self.stop()
                            
                            break

                        if self.csStatus == "search" and bStartCharging == False:
                            bStartCharging = True
                            print("Arrived at charging station position...")  
                            print("====>csStatus : " + self.csStatus)
                            self.navi_runstatstr.emit("Searching.....")

                            # # 2022.07. 27
                            self.setlaser.emit(False)

                            self.setuvblamp.emit(False)                           

                        if self.csStatus == "contact":
                            print("====>csStatus : " + self.csStatus)
                            self.navi_runstatstr.emit("Complete parking charging station.")

                            # 2022. 03. 03
                            self.setapflag.emit(True)
                            # self.setrunflag.emit(False)  
                            
                            # 2022. 03. 03
                            self.setpcsbtn.emit(False)  
                            print("change Parking C.S button...")               
                            
                            break
                            
                        sleep(0.1)
                
            else:
                pass

    def run(self):
        for i in range(1, 15):
            self.HDIRun[i] = 0

        if self.bRunFlag == True:
            self.navi_Process()

        self.stop()

    def stop(self):
                       
        # self.nctrlstatus_sub.unregister()
        # self.dsrstatus_sub.unregister()
        # self.chargestatus_sub.unregister()
        # self.estop_sub.unregister()

        if self.cap.isOpened():
            self.cap.release()
            print("Release camera resource...")

        self.quit()
        self.wait()

            
class Thread_ImageProcessing(QThread):
    VideoSignal2 = pyqtSignal(QImage)
    VideoSignal3 = pyqtSignal(QImage)
    
    print_mess = pyqtSignal(str)
    clearscene = pyqtSignal()
    # setbtn = pyqtSignal(bool)
    pubtwist = pyqtSignal(float)
    endadj = pyqtSignal()

    def __init__(self, parent = None):
        super(Thread_ImageProcessing, self).__init__(parent)
        self.working = True

        self.cvImg = None
        self.grayImg = None
        self.rotImg = None
        self.flipImg = None
        self.qtImg = None
        self.scaledImg = None
        self.originImg = None
        self.bGetImgOK = False
        self.bImgProcOK = False
        self.bAdjustPos = False
        self.bMoving = False
        self.bMobiling = False
        self.bCAMAdapt = False

        self.movedir = 0
        self.mov_gainy = 0.2
        self.mov_gainz = 0.3
        self.diffconstx = 20.0
        self.diffconsty = 25.0
        self.diffangle = 3.0
        self.initangle = 15.0
        self.fmoveval1 = 220.0
        self.fmoveval2 = 70.0
        self.finaloffset = -10.0
        self.movdiff = 30.0
        self.AdjStep = 1
        self.readAngCnt = 0
        self.getPosVal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.area = 0
        self.bGetPosOk = False
        self.diffx = 0
        self.diffy = 0
        self.prev_diffx = 0
        self.prev_diffy = 0
        self.movcnt = 100
        self.divangle = 2.0

        self.trajStatus = ZBDSRTrajStatus.IDLING

        self.cvBridge = CvBridge()

        self.pub_movpos = rospy.Publisher("/control_dsr_mov", ZBDSRMove, queue_size = 10)
        self.dsrstatus = rospy.Subscriber("/status_dsr_traj", ZBDSRTrajStatus, self.CallbackRunTrajStatus)
        self.senddsrpos_sub = rospy.Subscriber("/send_dsr_posx", DSRPos, self.CallSendPos) 
        self.pub_getdsrpos = rospy.Publisher("/get_dsr_posx", Bool, queue_size = 10)

        # 2022. 08. 25
        vindex = self.video_detect_index()
        print("video device index : ", vindex)

        self.cap = cv2.VideoCapture(vindex)

    def video_detect_index(self):
        video_idx = None

        for video_idx_file in os.listdir("/sys/class/video4linux"):
            video_file_path = os.path.realpath("/sys/class/video4linux/" + video_idx_file)

            for video_files in os.listdir(video_file_path):
                if 'name' in video_files:
                    name = open(os.path.realpath(video_file_path + '/' + 'name'), 'r').readline()
                    print("name:" + name)

                    if re.match('FULL HD', name) != None:
                        index = open(os.path.realpath(video_file_path + '/' + 'index'), 'r').readline()
                        print("index:")
                        print(index)
                        if re.match('0', index) != None:
                            video_idx = "/dev/" + video_idx_file
                            print(video_idx)

        return video_idx

    def CallSendPos(self, msg):

        self.getPosVal[0] = msg.px
        self.getPosVal[1] = msg.py
        self.getPosVal[2] = msg.pz
        self.getPosVal[3] = msg.rx
        self.getPosVal[4] = msg.ry
        self.getPosVal[5] = msg.rz

        print("getPosval:" + str(self.getPosVal))

        self.bGetPosOk = True


    def SetAdjustFlag(self, val):
        self.bAdjustPos = val


    def RunRobotMove(self):
        zbdsr_move = ZBDSRMove()

        zbdsr_move.xpos = 661.23
        zbdsr_move.ypos = -0.31 + 50
        zbdsr_move.zpos = 499.15 + 50
        zbdsr_move.rxpos = 1.53
        zbdsr_move.rypos = 91.58
        zbdsr_move.rzpos = 180.0
        zbdsr_move.setstatus = ZBDSRMove.START

        self.pub_movpos.publish(zbdsr_move)

        mstr = "Start DSR Robot Moving Pos #1"
        self.print_mess.emit(mstr)

        while True:

            if self.trajStatus == ZBDSRMove.COMPLETED:
                break

            sleep(0.1)

        zbdsr_move = ZBDSRMove()

        zbdsr_move.xpos = 661.23
        zbdsr_move.ypos = -0.31
        zbdsr_move.zpos = 499.15
        zbdsr_move.rxpos = 1.53
        zbdsr_move.rypos = 91.58
        zbdsr_move.rzpos = 180.0
        zbdsr_move.setstatus = ZBDSRMove.START

        self.pub_movpos.publish(zbdsr_move)

        mstr = "Start DSR Robot Moving Pos #2"
        self.print_mess.emit(mstr)

        while True:

            if self.trajStatus == ZBDSRMove.COMPLETED:
                break

            sleep(0.1)

        mstr = "End Moving Pos of DSR Robot..."
        self.print_mess.emit(mstr)


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
        self.print_mess.emit(mstr)

    def CallHCamRGB(self, msgs):
        global bCAMAdapt

        try:
            if bCAMAdapt == True:

                self.cvImg = self.cvBridge.imgmsg_to_cv2(msgs, "bgr8")            

                self.rgbImg = cv2.cvtColor(self.cvImg, cv2.COLOR_BGR2RGB)        
                self.grayImg = cv2.cvtColor(self.cvImg, cv2.COLOR_BGR2GRAY)               

                if self.bImgProcOK == False:
                    self.bGetImgOK = True    

                # self.scene1.addPixmap(QPixmap.fromImage(self.scaledImg))
                # self.camcap_graphicsView.setScene(self.scene1)

        except:
            print("CAM capture error!!!!")
            pass
                    
    def run(self):
        
        self.bEndProc = False

        cx = 0
        cy = 0
        self.diffx = 0
        self.diffy = 0

        prev_xpos = 487.03
        prev_ypos = -44.540
        prev_zpos = 525.39
        prev_rxpos = 179.14
        prev_rypos = -93.07
        prev_rzpos = 0.2

        angle = 0
        self.AdjStep = 1

        # cmdstr = "gnome-terminal -- rostopic pub /get_dsr_posx std_msgs/Bool \"data: true\" -1 "
        # os.system(cmdstr)

        sval = Bool()
        sval.data = True
        self.pub_getdsrpos.publish(sval)

        self.repcnt = 0
        self.bMobiling = False
        self.caperr_cnt = 0

        while True:
            
            if self.bEndProc is True:
                    break

            self.bGetImgOK = True

            if self.bGetImgOK == True and self.bImgProcOK == False:
                self.bImgProcOK = True

                ret, self.cvImg = self.cap.read()

                if ret is False:
                    print("can't open video!!")

                    self.caperr_cnt += 1

                    if self.caperr_cnt > 10:
                        self.caperr_cnt = 0

                        vindex = self.video_detect_index()
                        print("video device index : ", vindex)

                        self.cap = cv2.VideoCapture(vindex)

                        pass
                else:
                    self.rgbImg = cv2.cvtColor(self.cvImg, cv2.COLOR_BGR2RGB)        
                    self.grayImg = cv2.cvtColor(self.cvImg, cv2.COLOR_BGR2GRAY)

                    self.rotImg = cv2.rotate(self.rgbImg, cv2.ROTATE_90_CLOCKWISE)
                    self.flipImg = self.rotImg.copy()

                    h, w, ch = self.flipImg.shape
                    bytesPerLine = ch * w

                    self.centerx = w/2
                    self.centery = h/2

                    self.rgbImg2 = self.flipImg.copy()
                    self.grayImg = cv2.rotate(self.grayImg, cv2.ROTATE_90_CLOCKWISE)
                    self.grayImg = cv2.GaussianBlur(self.grayImg, (7,7), 0)
                
                    edged = cv2.Canny(self.grayImg, 50, 100)
                    edged = cv2.dilate(edged, None, iterations=1)                
                    edged = cv2.erode(edged, None, iterations=1)
                    
                    # find contours in the edge map
                    contours = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    cnts = imutils.grab_contours(contours)
                    cv2.drawContours(self.rgbImg2, cnts, -1, (0, 255, 0), 4)

                    if self.bAdjustPos == True and self.bMoving == False:
                        areas = [cv2.contourArea(c) for c in cnts]
                        try:
                            max_index = np.argmax(areas)
                            cnt=cnts[max_index]

                            self.area = areas[max_index]
                            print("max index:" + str(max_index) + " max areas:" + str(areas[max_index]))

                            #  2022. 08. 09
                            ellipse = cv2.fitEllipse(cnt)
                            angle = 90 - ellipse[2]

                            print("angle : " + str(angle))

                            self.readAngCnt += 1
                        except:
                            angle = 0

                        if self.area > 15000:
                            try:
                                x, y, wd, he = cv2.boundingRect(cnt)
                                cv2.rectangle(self.rgbImg2 , (x,y), (x+wd,y+he), (0,0,255), 10)
                                
                                cx = x + wd/2
                                cy = y + he/2

                                cv2.circle(self.rgbImg2 , (cx, cy), 20, (0, 0, 255), -1)

                                self.centery += self.finaloffset

                                self.diffx = self.centerx - cx
                                self.diffy = self.centery - cy  
                            except:
                                self.diffx = 0
                                self.diffy = 0

                                pass
                        else:
                            cx = 0
                            cy = 0
                            self.diffx = 0
                            self.diffy = 0

                        print("cx:", cx, " cy:", cy)                  
                        print("diffx:", self.diffx, " diffy:", self.diffy)                  

                    if self.bAdjustPos == True and self.bMobiling == False and self.AdjStep == 1:
                        print("cx:", cx, " fabs(diffx):", math.fabs(self.diffx))
                        if cx <= 0 or (math.fabs(self.diffx) >= 300):
                            self.pubtwist.emit(0.03) 
                            self.movedir = 1        
                            self.bMobiling = True     
                            self.AdjStep = 2           
                            self.repcnt = 0
                            self.movcnt = 100
                            print("Start moving foarward...")
                        else:
                            self.AdjStep = 3
                            self.bMobiling = True 

                    if self.bAdjustPos == True and self.bMobiling == True and self.AdjStep == 2:    # and self.area > 15000:    # and self.repcnt < 30:
                        if cx <= 0:
                            self.repcnt += 1
                            if self.movedir == 0 and self.repcnt > self.movcnt:
                                self.pubtwist.emit(0.03) 
                                self.movedir = 1
                                self.repcnt = 0
                                self.movcnt = 200
                                print("Return forward...")
                            elif self.movedir == 1 and self.repcnt > self.movcnt:
                                self.pubtwist.emit(-0.03) 
                                self.movedir = 0
                                self.repcnt = 0
                                self.movcnt = 300
                                print("Return backward...")
                        elif math.fabs(self.diffx) < 100:
                            self.AdjStep = 3
                            print("repcnt :" + str(self.repcnt))
                            self.repcnt = 0
                            print("cx :" + str(cx) + " diffx: " + str(self.diffx))


                    if self.bAdjustPos == True and self.bMobiling == True and self.AdjStep == 3:
                        if self.diffx >= 0:
                            self.pubtwist.emit(-0.02) 
                        else:
                            self.pubtwist.emit(0.02) 

                        self.AdjStep = 4
                        self.bMobiling = True

                        self.prev_diffx = self.diffx

                        print("Start adjustment mobiling...")
                        sleep(1)

                    elif self.bAdjustPos == True and self.bMobiling == True and self.AdjStep == 4:
                        if self.diffx <= self.movdiff and self.diffx >= -1.0*self.movdiff:    
                            self.pubtwist.emit(0.0)

                            self.AdjStep = 5
                            self.bMobiling = False
                            print("Done adjustment mobiling...")
                            print(" diffx: " + str(self.diffx))
                            sleep(1)
                        elif self.diffx < -1.0*self.movdiff and self.prev_diffx >= 0:
                            self.pubtwist.emit(0.02)
                            self.prev_diffx = self.diffx
                            print("Exceed forward x-axis moving...")
                            sleep(1)
                        elif self.diffx > self.movdiff and self.prev_diffx <= 0:
                            self.pubtwist.emit(-0.02)
                            self.prev_diffx = self.diffx
                            print("Exceed backward x-axis moving...")
                            sleep(1)

                    if self.bAdjustPos == True and self.bMoving == False and self.AdjStep == 5: 
                        zbdsr_move = ZBDSRMove()
                        zbdsr_move.xpos = prev_xpos

                        if math.fabs(self.diffy) > self.diffconsty:
                            dval = self.diffy*self.mov_gainz
                            zbdsr_move.zpos = prev_zpos + dval

                            zbdsr_move.rxpos = prev_rxpos
                            zbdsr_move.rypos = prev_rypos
                            zbdsr_move.rzpos = prev_rzpos
                            zbdsr_move.setstatus = ZBDSRMove.START

                            self.pub_movpos.publish(zbdsr_move)

                            self.trajStatus = ZBDSRTrajStatus.IDLING
                            prev_ypos = zbdsr_move.ypos
                            prev_zpos = zbdsr_move.zpos
                            prev_rzpos = zbdsr_move.rzpos
                            
                            self.bMoving = True

                            mstr = "DSR Robot Moving Pos(" + str(self.diffx) + ", " + str(self.diffy) + ")"
                            print(mstr)
                            self.print_mess.emit(mstr)

                            print("dx : " + str(self.diffx*self.mov_gainy) + "   dy : " +str(self.diffy*self.mov_gainz))

                        elif math.fabs(self.diffy) <= self.diffconsty:
                            self.AdjStep = 6
                            self.angleAdjCnt = 0
                            self.readAngCnt = 0
                            print("done adjustment x-axis")

                            sval = Bool()
                            sval.data = True
                            self.pub_getdsrpos.publish(sval)

                            self.bGetPosOk = False
                            print("get dsr position...")

                    if self.AdjStep == 6 and self.bMoving == False and self.bGetPosOk == True:
                        self.bGetPosOk = False

                        print("get pos :" + str(self.getPosVal[0]) + ", " + str(self.getPosVal[1]) + ", " + str(self.getPosVal[2]) + ", " + str(self.getPosVal[3]) + ", " + str(self.getPosVal[4]) + ", " + str(self.getPosVal[5]))

                        zbdsr_move.xpos = self.getPosVal[0] + self.fmoveval1
                        zbdsr_move.ypos = self.getPosVal[1]
                        zbdsr_move.zpos = self.getPosVal[2]
                        zbdsr_move.rxpos = self.getPosVal[3]
                        zbdsr_move.rypos = self.getPosVal[4]
                        zbdsr_move.rzpos = self.getPosVal[5]

                        zbdsr_move.setstatus = ZBDSRMove.START

                        self.pub_movpos.publish(zbdsr_move)

                        self.trajStatus = ZBDSRTrajStatus.IDLING

                        print("Send X-axis moving command...")

                        self.bMoving = True


                    # if self.bAdjustPos == True and self.bMoving == False and (self.AdjStep == 5 or self.AdjStep == 8): # and self.bEStop is False:

                    #     zbdsr_move = ZBDSRMove()
                    #     zbdsr_move.xpos = prev_xpos
                    #     if self.AdjStep == 5 or self.AdjStep == 8:
                    #         if math.fabs(self.diffx) > self.diffconstx:
                    #             zbdsr_move.ypos = prev_ypos + self.diffx*self.mov_gainy
                    #         else:
                    #             zbdsr_move.ypos = prev_ypos
                    #     else:
                    #         zbdsr_move.ypos = prev_ypos

                    #     if self.AdjStep == 5 or self.AdjStep == 8:
                    #         if math.fabs(self.diffy) > self.diffconsty:
                    #             dval = self.diffy*self.mov_gainz
                    #             zbdsr_move.zpos = prev_zpos + dval
                    #         else:
                    #             zbdsr_move.zpos = prev_zpos
                    #     else:
                    #         zbdsr_move.zpos = prev_zpos

                    #     if math.fabs(self.diffx) <= self.diffconstx and math.fabs(self.diffy) <= self.diffconsty:
                    #         if self.AdjStep == 5:
                    #             self.AdjStep = 6
                    #             self.angleAdjCnt = 0
                    #             self.readAngCnt = 0
                    #             print("done first-time adjust x-axis")
                    #         elif self.AdjStep == 8:
                    #             self.AdjStep = 9
                    #             print("done second-time adjust x-axis")

                    #             sval = Bool()
                    #             sval.data = True
                    #             self.pub_getdsrpos.publish(sval)

                    #             self.bGetPosOk = False
                    #             print("[2] get dsr position...")
                    #     else:
                    #         mstr = "Start DSR Robot Moving Pos(" + str(self.diffx) + ", " + str(self.diffy) + ")"
                    #         print(mstr)
                    #         self.print_mess.emit(mstr)

                    #         print("dx : " + str(self.diffx*self.mov_gainy) + "   dy : " +str(self.diffy*self.mov_gainz))

                    #     if self.AdjStep == 5 or self.AdjStep == 8:
                    #         zbdsr_move.rxpos = prev_rxpos
                    #         zbdsr_move.rypos = prev_rypos
                    #         zbdsr_move.rzpos = prev_rzpos
                    #         zbdsr_move.setstatus = ZBDSRMove.START

                    #         self.pub_movpos.publish(zbdsr_move)

                    #         self.trajStatus = ZBDSRTrajStatus.IDLING
                    #         prev_ypos = zbdsr_move.ypos
                    #         prev_zpos = zbdsr_move.zpos
                    #         prev_rzpos = zbdsr_move.rzpos
                            
                    #         self.bMoving = True

                    # if self.AdjStep == 7 and self.bMoving == False and self.bGetPosOk == True:
                    #     self.bGetPosOk = False

                    #     print("[step7] get pos :" + str(self.getPosVal[0]) + ", " + str(self.getPosVal[1]) + ", " + str(self.getPosVal[2]) + ", " + str(self.getPosVal[3]) + ", " + str(self.getPosVal[4]) + ", " + str(self.getPosVal[5]))

                    #     zbdsr_move.xpos = self.getPosVal[0]
                    #     zbdsr_move.ypos = self.getPosVal[1]
                    #     zbdsr_move.zpos = self.getPosVal[2]
                    #     zbdsr_move.rxpos = self.getPosVal[3]
                    #     zbdsr_move.rypos = self.getPosVal[4]
                    #     zbdsr_move.rzpos = self.getPosVal[5]
                    #     self.AdjStep = 8
                    #     self.bMoving = False


                    # if self.AdjStep == 9 and self.bMoving == False and self.bGetPosOk == True: # and self.bEStop is False:                        
                    #     self.bGetPosOk = False

                    #     print("get pos :" + str(self.getPosVal[0]) + ", " + str(self.getPosVal[1]) + ", " + str(self.getPosVal[2]) + ", " + str(self.getPosVal[3]) + ", " + str(self.getPosVal[4]) + ", " + str(self.getPosVal[5]))

                    #     zbdsr_move.xpos = self.getPosVal[0] + self.fmoveval1
                    #     zbdsr_move.ypos = self.getPosVal[1]
                    #     zbdsr_move.zpos = self.getPosVal[2]
                    #     zbdsr_move.rxpos = self.getPosVal[3]
                    #     zbdsr_move.rypos = self.getPosVal[4]
                    #     zbdsr_move.rzpos = self.getPosVal[5]

                    #     zbdsr_move.setstatus = ZBDSRMove.START

                    #     # prev_xpos = zbdsr_move.xpos
                    #     # prev_ypos = zbdsr_move.ypos
                    #     # prev_zpos = zbdsr_move.zpos
                    #     # prev_rxpos = zbdsr_move.rxpos
                    #     # prev_rypos = zbdsr_move.rypos
                    #     # prev_rzpos = zbdsr_move.rzpos

                    #     self.pub_movpos.publish(zbdsr_move)

                    #     self.trajStatus = ZBDSRTrajStatus.IDLING

                    #     print("Send X-axis moving command...")

                    #     self.bMoving = True

                    # if self.AdjStep == 6 and self.bMoving == False:
                    #     print("readAngCnt:" + str(self.readAngCnt))
                    #     if self.readAngCnt >= 3:
                    #         if math.fabs(angle) >= self.diffangle:
                    #             self.readAngCnt = 0
                    #             if math.fabs(angle) > self.initangle:
                    #                 if angle > 0.0:
                    #                     angle = self.initangle
                    #                 else:
                    #                     angle = -1.0*self.initangle

                    #             zbdsr_move.rzpos = prev_rzpos  - angle/self.divangle

                    #             zbdsr_move.rxpos = prev_rxpos
                    #             zbdsr_move.rypos = prev_rypos

                    #             if self.angleAdjCnt >= 10:
                    #                 zbdsr_move.rzpos = prev_rzpos
                    #                 # self.AdjStep = 7
                    #                 self.angleAdjCnt = 0
                    #                 self.divangle *= 1.5
                    #                 print("time out angle adj....")
                    #             else:
                    #                 self.angleAdjCnt += 1

                    #                 zbdsr_move.setstatus = ZBDSRMove.START
                    #                 self.pub_movpos.publish(zbdsr_move)
                    #                 self.trajStatus = ZBDSRTrajStatus.IDLING

                    #                 print("math.fabs(angle) : " + str(math.fabs(angle)))
                    #                 print("rotate angle:" + str(angle) + " prev rzpos:" +str(prev_rzpos))
                    #                 print("Send Z-axis moving command...")

                    #                 self.bMoving = True

                    #         else:
                    #             zbdsr_move.rzpos = prev_rzpos
                    #             self.AdjStep = 7
                    #             self.readAngCnt = 0

                    #             print("angle:" + str(angle))
                    #             print("done adjust rz-axis")

                    #             sval = Bool()
                    #             sval.data = True
                    #             self.pub_getdsrpos.publish(sval)

                    #             self.bGetPosOk = False
                    #             print("[1] get dsr position...")
                                
                    #     else:
                    #         zbdsr_move.rzpos = prev_rzpos


                    if self.bAdjustPos == True and self.bMoving == True and self.trajStatus == ZBDSRTrajStatus.COMPLETED: # and self.bEStop is False:
                        sleep(0.5)

                        self.trajStatus = ZBDSRTrajStatus.IDLING

                        self.bMoving = False

                        if self.AdjStep == 6:
                            self.AdjStep = 7
                            # self.clearscene.emit()

                        mstr = "End Moving Pos of DSR Robot..."
                        print(mstr)
                        self.print_mess.emit(mstr)
                    
                    if self.bAdjustPos == True:
                        cv2.putText(self.rgbImg2, 'cx,cy=({0}, {1})'.format(self.centerx, self.centery), (50, 420+300), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 255, 0), 5)
                        cv2.putText(self.rgbImg2, 'x,y=({0}, {1})'.format(cx, cy), (50, 420+450), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 0, 0), 5)
                        cv2.putText(self.rgbImg2, 'dx,dy=({0}, {1})'.format(self.diffx, self.diffy), (50, 420+600), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 0, 255), 5)
                        cv2.putText(self.rgbImg2, 'angle= {0}'.format(angle), (50, 420+750), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 255, 255), 5)

                        h2, w2, ch2 = self.rgbImg2.shape
                        bytesPerLine2 = ch2 * w2

                        qt_image = QImage(self.rgbImg2.data, w2, h2, bytesPerLine2, QImage.Format_RGB888)
                        p = qt_image.scaled(151, 201, Qt.KeepAspectRatio)
                        self.VideoSignal3.emit(p)

                    self.qtImg = QImage(self.flipImg.data, w, h, bytesPerLine, QImage.Format_RGB888)
                    self.scaledImg = self.qtImg.scaled(151, 201, Qt.KeepAspectRatio)
                    self.VideoSignal2.emit(self.scaledImg)

                    self.bImgProcOK = False

                if self.AdjStep == 7:                
                # if self.AdjStep == 10:   
                    break

                sleep(0.1)

        self.clearscene.emit()

        self.endadj.emit()

        print("Exit Image Processing Thread...")


    def stop(self):
        self.bEndProc = True
        
    def procExit(self):
        # self.hcam_sub.unregister()
        # self.dsrstatus.unregister()

        # self.endadj.emit()
        
        self.bEndProc = True

        sleep(1)

        if self.cap.isOpened():
            self.cap.release()
            print("Release camera resource...")


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

        

class CAMViewWindow(QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Camera Viewer Window')
 
        self.online_webcams = QCameraInfo.availableCameras()
        if self.online_webcams:   
            self.exist = QCameraViewfinder()
            self.exist.show()
            self.get_webcam(5)
            
        else:
            self.print_MessStr("The web camera is not installed.")

        self.btnOK = QPushButton("OK")
        self.btnOK.clicked.connect(self.onOKButtonClicked)

        self.Layout = QVBoxLayout()
        self.Layout.addWidget(self.exist)
        self.Layout.addWidget(self.btnOK)
        self.setLayout(self.Layout)

        self.setFixedWidth(640)
        self.setFixedHeight(480)
      

    def get_webcam(self, i):
        self.webcam = QCamera(self.online_webcams[i])
        self.webcam.setViewfinder(self.exist)
        self.webcam.setCaptureMode(QCamera.CaptureStillImage)
        self.webcam.error.connect(lambda: self.alert(self.my_webcam.errorString()))
        self.webcam.start()

    def alert(self, s):
        """
        This handle errors and displaying alerts.
        """
        err = QErrorMessage(self)
        err.showMessage(s)        

    def onOKButtonClicked(self):
        self.close()
    


class Thread_RunReady(QThread):
    runpackage = pyqtSignal(int)
    exitpackage = pyqtSignal(int)
    setrunstate = pyqtSignal(int)
 
    def __init__(self, parent = None):
        super(Thread_RunReady, self).__init__(parent)

        self.type = 1
        
    def run(self):
        if self.type == 1 :
            self.runpackage.emit(1)
            sleep(9)
            self.setrunstate.emit(13)

            self.runpackage.emit(2)
            sleep(4)
            self.setrunstate.emit(25)            

            self.runpackage.emit(3)
            sleep(5)
            self.setrunstate.emit(38)

            self.runpackage.emit(4)
            sleep(8)
            self.setrunstate.emit(50)

            self.runpackage.emit(5)
            sleep(6)
            self.setrunstate.emit(63)

            self.runpackage.emit(6)
            sleep(6)
            self.setrunstate.emit(75)

            self.runpackage.emit(7)
            sleep(6)
            self.setrunstate.emit(88)

            self.runpackage.emit(8)
            sleep(6)
            self.setrunstate.emit(100)
            
            print("Done ready...")

        else:
            self.exitpackage.emit(8)
            sleep(4)
            self.setrunstate.emit(13)

            self.exitpackage.emit(7)
            sleep(4)
            self.setrunstate.emit(25)
            
            self.exitpackage.emit(6)
            sleep(4)
            self.setrunstate.emit(38)
            
            self.exitpackage.emit(5)
            sleep(5)
            self.setrunstate.emit(50)
            
            self.exitpackage.emit(4)
            sleep(3)
            self.setrunstate.emit(63)
            
            self.exitpackage.emit(3)
            sleep(6)
            self.setrunstate.emit(75)

            self.exitpackage.emit(2)
            sleep(6)
            self.setrunstate.emit(88)

            self.exitpackage.emit(1)
            sleep(6)
            self.setrunstate.emit(100)
            
            print("Done init...")

    def setType(self, val):
        self.type = val



class Thread_RunSchedule(QThread):
    runtrajloop = pyqtSignal()
    runcsparking = pyqtSignal()    

    def __init__(self, parent = None):
        super(Thread_RunSchedule, self).__init__(parent)
        

    def run(self):
        global bRunSchedule

        start_time = 0
        end_time = 24

        inc_time = 30

        hour_iter = (end_time - start_time) + 1
        minute_iter = int(60/inc_time) + 1
        print("hour_iter:{}  minute_iter:{}".format(hour_iter, minute_iter))


        cur_time = start_time
        cur_minute = 0
        
        for i in range(1, hour_iter):
            for j in range(1, minute_iter):
                time_str = str(cur_time).zfill(2) + ":" + str(cur_minute).zfill(2)

                if j % 2 == 1:
                    print("Traj time :" + time_str)
                    schedule.every().day.at(time_str).do(self.RunTrajLoop)
                else:
                    print("Parking time :" + time_str)
                    schedule.every().day.at(time_str).do(self.RunCSParking)

                cur_minute += inc_time

            cur_minute = 0
            cur_time += 1

        if cur_time >= 24:
            cur_time = 0
            cur_minute = 1
        
        time_str = str(cur_time).zfill(2) + ":" + str(cur_minute).zfill(2)

        schedule.every().day.at(time_str).do(self.StopSchedule)


        # schedule.every().day.at("15:45").do(self.RunTrajLoop)
        # schedule.every().day.at("15:50").do(self.RunCSParking)

        # schedule.every().day.at("23:10").do(self.RunTrajLoop)
        # schedule.every().day.at("23:20").do(self.RunCSParking)

        # schedule.every().day.at("23:30").do(self.RunTrajLoop)
        # schedule.every().day.at("23:40").do(self.RunCSParking)

        # schedule.every().day.at("23:50").do(self.RunTrajLoop)
        # schedule.every().day.at("00:00").do(self.RunCSParking)

        # schedule.every().day.at("00:10").do(self.RunTrajLoop)
        # schedule.every().day.at("00:20").do(self.RunCSParking)

        # schedule.every().day.at("00:30").do(self.RunTrajLoop)
        # schedule.every().day.at("00:40").do(self.RunCSParking)

        # schedule.every().day.at("00:50").do(self.RunTrajLoop)
        # schedule.every().day.at("01:00").do(self.RunCSParking)

        # schedule.every().day.at("01:10").do(self.RunTrajLoop)
        # schedule.every().day.at("01:20").do(self.RunCSParking)


        while bRunSchedule == True:
            schedule.run_pending()
            sleep(1)

        print("Exit schedule thread...")


    def StopSchedule(self):
        global bRunSchedule

        schedule.cancel_job(self.RunTrajLoop)
        schedule.cancel_job(self.RunCSParking)

        schedule.clear(tag=None)

        bRunSchedule = False

        print("Stopping schedule...")

    def RunTrajLoop(self):
        self.runtrajloop.emit()

    def RunCSParking(self):
        self.runcsparking.emit()


class MyWindow(QMainWindow, form_class):
    def __init__(self):
        global HDSch_List
        global CRDSch_List
        global bUseCAMAdj

        # global schlist_cnt

        super(QMainWindow, self).__init__()
        self.setupUi(self)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

        self.settings = termios.tcgetattr(sys.stdin)

        self.bRoscoreRun = False
        self.bBringUpRun = False
        self.bFrontWF = False
        self.bRearWF = False
        self.bFrontSWF = False
        self.bRearSWF = False
        self.bCRDIFrontSWF = False
        self.bCRDIRearSWF = False
        self.bSLAMRun = False
        self.bTeleOPRun = False
        self.b2LRFRun = False
        self.bNaviRun = False
        self.bRVIZRun = False
        self.bWPCRun = False
        self.bMakeWPRun = False
        self.bDSRRobotRun = False
        self.bDSRTrajRun = False
        self.bDSRStart = False
        self.bStopStart = False
        self.bReadyRun = False
        self.bReadyDone = False
        self.dsrServiceConOK = False
        self.cur_trajnum = 0
        self.bCSParking = False
        self.bmotionCtrl = False
        self.bLauchCAMTopic = False
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
        self.bSLAMRVIZRun = False
        self.bRunWebManager = False
        self.bCAMView = False
        self.bOPRun = False
        self.bViewSonarData = False
        self.bRunNaviWP = False
        self.bRunTrajNavi = False
        self.bGetCurPos = False
        self.bEStop = False
        self.bprevEStop = False
        self.bEStop_Status = 1

        self.bRunMovPos = False

        # self.bCAMAdapt = False
        
        self.naviStatus = NavigationControlStatus.IDLING

        self.SelectWF = 0b00
        self.SelWFRange = 0b11
        self.SelectCRDIWF = 0b00
        self.SelCRDIWFRange = 0b11
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
        self.IMUVCnt = 0
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0

        self.VSDCnt = 0
        
        self.schedule_run_cnt = 0       
        
        self.robot_mode = "None"


        self.twolrf_pro = []
        self.rosbringup_proc = []
        self.oprobot_proc = []
        self.autocharge_proc = []
        self.gridmap_proc = []
        self.slamrviz_proc = []
        self.navigation_proc = []
        self.navirviz_proc = []
        self.wpctrl_proc = []
        self.makewaypoint_proc = []
        self.teleop_proc = []
        self.dsrrobot_proc = []
        self.dsrtraj_proc = []

        try :
            with open(yaml_fname) as f:
                wp_list = yaml.safe_load(f)
                # print(wp_list)
                
                wps = wp_list['waypoints']
                # print("waypoints")
                for n in wps:
                    # print("name : " + n["name"])
                    self.wp_CB.addItem(n["name"])

                # print("trajectories")
                trajs = wp_list['trajectories']
                for t in trajs:
                    # print("name : " + t["name"])
                    self.traj_CB.addItem(t["name"])

            for i in range(1, 14):
                trajname =  "Trajectory {0}".format(i)
                # print(trajname)
                self.dsrtraj_CB.addItem(trajname)
        except :
            bHDIWPFileLoadOK = False
            
        finally:
            bHDIWPFileLoadOK = True


        schlist_cnt = 0
        
        try :
            with open(schedule_fname) as f:
                schedule_list = yaml.safe_load(f)
                print(schedule_list)
                
                HDSch_List = schedule_list['Hallway_Disinfection']

                print("Hallway Disinfection")
                for m in HDSch_List:
                    print("----------------------")
                    print("name : " + m["name"])
                    print("type : " + m["type"])
                    print("start point : " + m["start_point"])
                    print("adjust point : " + m["adjust_point"])
                    print("dsrftnum : " + str(m["dsrftnum"]))
                    print("dsrbtnum : " + str(m["dsrbtnum"]))
                    print("end point : " + m["end_point"])
                    print("----------------------")   
                    
                    schlist_cnt += 1         
                    
                print("HD schedule list count : " + str(schlist_cnt))
                
                CRDSch_List = schedule_list['ConfRoom_Disinfection']

                print(schedule_list)
                
                schlist_cnt = 0
                print("Conference Room Disinfection")
                for n in CRDSch_List:
                    print("----------------------")
                    print("name : " + n["name"])
                    print("type : " + n["type"])
                    print("start point : " + n["start_point"])
                    print("dsrftnum : " + str(n["dsrftnum"]))
                    print("dsrbtnum : " + str(n["dsrbtnum"]))
                    print("water_pump : " + str(n["water_pump"]))
                    print("uvlamp : " + str(n["uvlamp"]))
                    print("----------------------")   
                    
                    schlist_cnt += 1         
                    
                print("CRD schedule list count : " + str(schlist_cnt))
                
        except :
            bScheduleLoadOK = False
            
        finally:
            bScheduleLoadOK = True
            
        
        self.messLV = QStandardItemModel()

        self.ncpub = rospy.Publisher("navi_ctrl", NavigationControl, queue_size = 10)
        self.dsrpub = rospy.Publisher("control_dsr_traj", ZBDSRSetTraj, queue_size = 10)
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

        self.put_ignoreWF = rospy.Publisher("/IgnoreWarningField", UInt8, queue_size = 5)
        self.put_warningFS = rospy.Publisher("/WarningFieldSelect", UInt8, queue_size = 5)

        self.rcspub = rospy.Publisher("robot_control_status", RobotControlStatus, queue_size = 10)
        
        
        self.trajstat_sub = rospy.Subscriber("/status_dsr_traj", ZBDSRTrajStatus, self.CallbackRunTrajStatus)
        self.navictrl_status_sub = rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)
        self.acstat_sub = rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.recv_IMU)
        self.robotpos_sub = rospy.Subscriber("/set_camview", Bool, self.CallSetCAMView)
        self.robotpos_sub = rospy.Subscriber("/robot_pose", Pose, self.CallCurPoseInfo)
        self.charge_state_nuc_sub = rospy.Subscriber("/autocharge_state_NUC", UInt8, self.CallbackCSStatusNUC)
        self.sonar_sub = rospy.Subscriber("/sonar", Float32MultiArray, self.recv_Sonar)      
        self.estop_sub = rospy.Subscriber("/EmergencyStop", UInt8, self.CallEStop) 
        

        # self.estop_sub = rospy.Subscriber("/estop", Bool, self.CallEStop)  

        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)

        # self.pub_movpos = rospy.Publisher("/control_dsr_mov", ZBDSRMove, queue_size = 10)
        
        self.wpCBStr = self.wp_CB.currentText()
        self.trajCBStr = self.traj_CB.currentText()
        self.dsrtrajCBStr = self.dsrtraj_CB.currentText()
        self.wp_CB.activated[str].connect(self.onActivatedwpCB)
        self.traj_CB.activated[str].connect(self.onActivatedtrajCB)
        self.dsrtraj_CB.activated[str].connect(self.onActivateddsrtrajCB)        

        self.wp_startPB.clicked.connect(self.onWPStartBtn)
        self.wp_stopPB.clicked.connect(self.onWPStopBtn)
        self.dsrtraj_runPB.clicked.connect(self.onDSRTrajRunBtn)
        self.dsrtraj_stopstartPB.clicked.connect(self.onDSRTrajStopStartBtn)
        self.traj_startPB.clicked.connect(self.onTrajStartBtn)
        self.traj_stopPB.clicked.connect(self.onTrajStopBtn)
        self.run_roscorePB.clicked.connect(self.onROSCoreBtn)
        self.run_bringupPB.clicked.connect(self.onBringupBtn)
        self.run_slamPB.clicked.connect(self.onSlamBtn)
        self.run_teleopPB.clicked.connect(self.onTeleopBtn)
        self.run_2LRFPB.clicked.connect(self.on2LFRBtn)
        self.run_OPPB.clicked.connect(self.onOperationBtn)
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
        self.run_dsrTrajPB.clicked.connect(self.onDSRTrajBtn)
        # self.run_connectSerial1PB.clicked.connect(self.onCoonectSerial1Btn)
        # self.run_connectSerial2PB.clicked.connect(self.onCoonectSerial2Btn)
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
        self.readyPB.clicked.connect(self.onReadyBtn)
        self.runhdiPB.clicked.connect(self.onHDIBtn)
        self.runcrdiPB.clicked.connect(self.onCRDIBtn)
        self.greenLaserCtrlPB.clicked.connect(self.onGreenLaserCtrlBtn)
        self.redLaserCtrlPB.clicked.connect(self.onRedLaserCtrlBtn)
        self.mcinit_PB.clicked.connect(self.InitMCValueBtn)
        self.getamclpos_PB.clicked.connect(self.onGetAMCLPosBtn)
        self.camtopic_PB.clicked.connect(self.onLaunchCAMTopic) 
        self.camView_PB.clicked.connect(self.onCAMViewBtn)
        self.webmanager_PB.clicked.connect(self.onWebManagerLaunch)
        self.initPos_PB.clicked.connect(self.onInitPosition)

        self.camCalib_PB.clicked.connect(self.onCalibrationBtn)
        self.camApply_PB.clicked.connect(self.onCAMApplyBtn)

        self.mrMov_PB.clicked.connect(self.onRobotMoveBtn)
        
        self.runschedulePB.clicked.connect(self.onRunScheduleBtn)
        
        
        self.greenLaserCtrlPB.setText('G-Laser Off')        
        self.redLaserCtrlPB.setText('R-Laser Off')

        self.lspeed_lineEdit.setText("0.0")
        self.aspeed_lineEdit.setText("0.0")
        self.lsinc_lineEdit.setText(str(initSpeed))
        self.asinc_lineEdit.setText(str(initTurn))

        self.trajmode = NavigationControl.NONE

        self.trajmode_checkBox.stateChanged.connect(self.changeTrajMode)
        self.viewsonar_checkBox.stateChanged.connect(self.changeViewSD)

        self.useCAMAdj_checkBox.setChecked(True)
        self.useCAMAdj_checkBox.stateChanged.connect(self.changeUseCAMAdj)
        bUseCAMAdj = True

        self.u1_progressBar.setFormat("%p mm")
        self.u2_progressBar.setFormat("%p mm")
        self.l1_progressBar.setFormat("%p mm")
        self.l2_progressBar.setFormat("%p mm")
        self.b1_progressBar.setFormat("%p mm")
        self.b2_progressBar.setFormat("%p mm")
        self.r1_progressBar.setFormat("%p mm")
        self.r2_progressBar.setFormat("%p mm")
        
        # run 12 terminal window
        os.system("python3 ~/bin/target_term -set 21")
        self.print_MessStr("Run terminal windows.")

        # for i in range(1, 16):
        #     cmd = "python3 ~/bin/target_term -run "+ str(i) + (" echo terminal num :" + str(i))
        #     # print(cmd)
        #     os.system(cmd)

        #     sleep(0.5)

        data = os.environ["HOME"]+"/.term_list"
        self.t_term = open(data).read().splitlines()
        # print(self.t_term)

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

        self.chargingLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.estopLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        
        self.fran_lineEdit.setText("0")
        self.rran_lineEdit.setText("0")

        self.mapfile_lineEdit.setText('dgcity_ver2_1')
        self.WPfile_lineEdit.setText('HDIWP16')

        self.ready_progBar.setFormat("%p%")
        self.ready_progBar.setValue(0)


        # self.naviHDI_th = Thread_HallowDisfectionProcess(self)

        # self.naviHDI_th = Thread_HollowDisfection(self)

        # self.naviCRDI_th = Thread_ConfRoomDisfection(self)

        
        sleep(2)

        cmd = "python3 ~/bin/target_term -run 1 roslaunch zetabank_bringup zetabank_robot_serial.launch"
        self.rosserial_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

        self.print_MessStr("Run roslaunch zetabank_robot_serial.launch")

        sleep(2)
        
        # os.system("python3 ~/bin/target_term -run 21 rosrun flir_one_node flir_one_node ")

        # sleep(4)

        # self.cvBridge = CvBridge()

        # self.scene1 = QGraphicsScene()
        # self.scene2 = QGraphicsScene()

        # self.camThread = Thread_ImageProcessing(self)
        
        # self.camThread.VideoSignal2.connect(self.setImage2)
        # self.camThread.VideoSignal3.connect(self.setImage3)
        # self.camThread.print_mess.connect(self.messPrint3)
        # self.camThread.clearscene.connect(self.ClearScene)
        # self.camThread.setmovbtn.connect(self.SetMoveBtn)
       
        

        # sleep(2)

        # self.camThread.start()

        # self.hcam_sub = rospy.Subscriber("/camera_flir_node/rgb/image_raw", Image, self.CallHCamRGB) 

        self.bStartOK = True
        
        self.make_logfile()

        self.print_MessStr("Complete initialization...")

        # self.onUVCBottomOffBtn()


    def pubRobotcontrolStatus(self, stat):
        rcs = RobotControlStatus()
        rcs.status = stat
        
        if stat == RobotControlStatus.ERROR:
            rcs.status_description = "Robot OP Status : Error"
        elif stat == RobotControlStatus.IDLING:
            rcs.status_description = "Robot OP Status : Idling"
        elif stat == RobotControlStatus.NAVI:
            rcs.status_description = "Robot OP Status : Waypoint navigation"
        elif stat == RobotControlStatus.DSR:
            rcs.status_description = "Robot OP Status : Moving DSR robot"
        elif stat == RobotControlStatus.UV:
            rcs.status_description = "Robot OP Status : Using UV Lamp"
        elif stat == RobotControlStatus.SPARY:
            rcs.status_description = "Robot OP Status : Using Spary pump"
        elif stat == RobotControlStatus.UVSPARY:
            rcs.status_description = "Robot OP Status : Using Spary pump and UV Lamp"
        elif stat == RobotControlStatus.CHARGING:
            rcs.status_description = "Robot OP Status : Charging in progress"
            
        self.rcspub.publish(rcs) 

    # @pyqtSlot(int)
    # def getcurtraj(self, trajnum):
    #     trajnum = self.cur_trajnum

    

    # @pyqtSlot(bool)
    # def SetBtn1(self, val):
    #     if val == True:
    #         self.mrMov_PB.setText('Move Robot')
    #     else:
    #         self.mrMov_PB.setText('Stop Robot')

    @pyqtSlot(bool)
    def setPump(self, val):
        if val == True:
            self.onPumpOnPBtn()
        else:
            self.onPumpOffPBtn()

    @pyqtSlot(bool)
    def setSol(self, val):
        if val == True:
            self.onSolOnPBtn()
        else:
            self.onSolOffPBtn()

    @pyqtSlot(bool)
    def setMoveBtn(self, val):
        if val == True:
            self.mrMov_PB.setEnabled(True)
        else:
            self.mrMov_PB.setDisabled(True)

    @pyqtSlot(int)
    def ClearScene(self, val):
        if val == 1:
            self.scene1.clear()
        elif val == 2:
            self.scene2.clear()
        elif val == 3:
            self.scene1.clear()
            self.scene2.clear()

    @pyqtSlot()
    def CAMAdjEnd1(self):
        global bCAMAdapt

        self.camApply_PB.setText('CAM Apply')
        bCAMAdapt = False
        self.bRunMovPos = False
        if self.camThread:
            self.camThread.SetAdjustFlag(False)
            self.camThread.procExit()
        self.mrMov_PB.setText('Move Robot')

    @pyqtSlot()
    def CAMAdjEnd2(self):
        global bCAMAdapt

        self.camApply_PB.setText('CAM Apply')
        bCAMAdapt = False
        self.bRunMovPos = False
        if self.naviHDI_th:
            self.naviHDI_th.SetAdjustFlag(False)
        self.mrMov_PB.setText('Move Robot')

    @pyqtSlot()
    def ClearScene2(self):
        # self.scene1.clear()
        self.scene2.clear()

    @pyqtSlot(QImage)
    def setImage1(self, image):
        self.scene1.addPixmap(QPixmap.fromImage(image))
        self.camcap_graphicsView.setScene(self.scene1)
    
    @pyqtSlot(QImage)
    def setImage2(self, image):
        self.scene1.addPixmap(QPixmap.fromImage(image))
        self.camcap_graphicsView.setScene(self.scene1)

    @pyqtSlot(QImage)
    def setImage3(self, image):
        self.scene2.addPixmap(QPixmap.fromImage(image))
        self.camcap_graphicsView2.setScene(self.scene2)
        
    @pyqtSlot(str)
    def messPrint1(self, str):
        self.print_MessStr(str)
        
    @pyqtSlot(str)
    def messPrint2(self, str):
        self.print_MessStr(str)

    @pyqtSlot(str)
    def messPrint3(self, str):
        self.print_MessStr(str)
        
    @pyqtSlot()
    def RunTrajLoop(self):
        self.schedule_run_cnt += 1

        if self.bCSParking == True:
            self.onParkingChargingStationBtn()

            # sleep(1)

            # self.onInitPosition()

            # sleep(2)

            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 1\" "
            # os.system(cmdstr)
            # sleep(1)
            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 1\" "
            # os.system(cmdstr)
            # sleep(1)           
        
        if self.bHDIRun == True:
            self.onHDIBtn()
            print("Run HDI process..")
            
        if self.bCRDIRun == True:
            self.onCRDIBtn()
            print("Run CRDI process..")
           

        print("Run trajectory : #" + str(self.schedule_run_cnt))
        self.print_MessStr("Run trajectory ....")

    @pyqtSlot()
    def RunCSParking(self):
        self.onTrajStartBtn()

        # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
        # os.system(cmdstr)
        # sleep(1)
        # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
        # os.system(cmdstr)
        # sleep(1)

        if self.bCSParking == False:
            self.onParkingChargingStationBtn()

            self.print_MessStr("Run auto parking....")
        
        print("Run auto parking : #" + str(self.schedule_run_cnt))
        
        
    def onRunScheduleBtn(self):
        global bRunSchedule

        if bRunSchedule == False:
            bRunSchedule = True

            # self.trajmode_checkBox.setChecked(True)
            # self.chargingmode_checkBox.setChecked(True)

            # if self.trajmode == NavigationControl.GOAL:
            #     self.trajmode = NavigationControl.LOOP

            # if self.csmode == False:
            #     self.csmode = True

            self.schedule_th = Thread_RunSchedule(self)
            self.schedule_th.start()

            self.schedule_th.runtrajloop.connect(self.RunTrajLoop)
            self.schedule_th.runcsparking.connect(self.RunCSParking)

            self.runproc_PB.setText('Stop Proc')       
            self.print_MessStr("Run schedule...")


        else:

            self.schedule_th.StopSchedule()

            # bRunSchedule = False

            # self.trajmode_checkBox.setChecked(False)
            # self.chargingmode_checkBox.setChecked(False)

            # self.trajmode = NavigationControl.GOAL
            self.csmode = False

            self.runproc_PB.setText('Run Proc')       
            self.print_MessStr("Stop schedule...")
            

    def write_log(self):
        global logfp
        global logdata

        today_str = strftime('%Y_%m_%d', localtime(time()))

        logfile_dir = rootdir + "/log"
        logfile_name = logfile_dir + "/safety_func_log_" + today_str + ".txt"

        logfp = open(logfile_name, 'a')
        logdata["now_time"] = strftime('%Y_%m_%d', localtime(time())) + " " + str(localtime(time()).tm_hour) + ":" + str(localtime(time()).tm_min) + ":" + str(localtime(time()).tm_sec)
        logfp.write('%s SFN=%03d WRegion=%03d WMode=%03d RMode=%03d RStatus=%s X=%f Y=%f Z=%f\n' % (logdata["now_time"], logdata["safety_info"], logdata["working_region"],
                    logdata["working_mode"], logdata["robot_mode"], logdata["robot_status"], logdata["pos_x"], logdata["pos_y"], logdata["pos_z"]))
        logfp.close()            
            
    def make_logfile(self):
        global blogFileReady
        global logfp
        
        today_str = strftime('%Y_%m_%d', localtime(time()))

        logfile_dir = rootdir + "/log"
        logfile_name = logfile_dir + "/safety_func_log_" + today_str + ".txt"

        if os.path.isfile(logfile_name):
            print("Ready log file...")
        else:
            if not os.path.exists(logfile_dir):
                os.makedirs(logfile_dir)

            logfp = open(logfile_name, 'w')
            data = "time\tSafetyInfo\tworkingRegion\tWorkingMode\tRobotMode\tRobotStatus\tRobotPosX\tRobotPosY\tRobotPosZ"
            logfp.write(data)
            logfp.close()    
            
    def onRobotMoveBtn(self):
        if self.bRunMovPos == False:
            self.bRunMovPos = True
            if self.camThread:
                self.camThread.SetAdjustFlag(True)
            # elif self.naviHDI_th:
            #     self.naviHDI_th.SetAdjustFlag(True)
            self.mrMov_PB.setText('Stop Robot')

        else:
            self.bRunMovPos = False
            if self.camThread:
                self.camThread.SetAdjustFlag(False)
            # elif self.naviHDI_th:
            #     self.naviHDI_th.SetAdjustFlag(False)
            self.mrMov_PB.setText('Move Robot')

            self.scene2.clear()
            self.scene1.clear()

        # self.camThread.RunRobotMove()
    
    # def onRobotMoveBtn(self):
    #     zbdsr_move = ZBDSRMove()

    #     zbdsr_move.xpos = 661.23
    #     zbdsr_move.ypos = -0.31 + 50
    #     zbdsr_move.zpos = 499.15
    #     zbdsr_move.rxpos = 1.53
    #     zbdsr_move.rypos = 91.58
    #     zbdsr_move.rzpos = 180.0
    #     zbdsr_move.setstatus = ZBDSRMove.START

    #     self.pub_movpos.publish(zbdsr_move)

    #     mstr = "Start DSR Robot Moving Pos #1"
    #     self.print_MessStr(mstr)

    #     while True:

    #         if self.trajStatus == ZBDSRMove.COMPLETED:
    #             break

    #         sleep(0.1)

    #     zbdsr_move = ZBDSRMove()

    #     zbdsr_move.xpos = 661.23
    #     zbdsr_move.ypos = -0.31 - 100
    #     zbdsr_move.zpos = 499.15
    #     zbdsr_move.rxpos = 1.53
    #     zbdsr_move.rypos = 91.58
    #     zbdsr_move.rzpos = 180.0
    #     zbdsr_move.setstatus = ZBDSRMove.START

    #     self.pub_movpos.publish(zbdsr_move)

    #     mstr = "Start DSR Robot Moving Pos #2"
    #     self.print_MessStr(mstr)

    #     while True:

    #         if self.trajStatus == ZBDSRMove.COMPLETED:
    #             break

    #         sleep(0.1)

    #     mstr = "End Moving Pos of DSR Robot..."
    #     self.print_MessStr(mstr)
      
    
    def onCAMApplyBtn(self):
        global bCAMAdapt

        if bCAMAdapt == False:
            bCAMAdapt = True
            self.camApply_PB.setText('CAM NApp')

            self.scene1 = QGraphicsScene()
            self.scene2 = QGraphicsScene()

            self.camThread = Thread_ImageProcessing(self)
        
            self.camThread.VideoSignal2.connect(self.setImage2)
            self.camThread.VideoSignal3.connect(self.setImage3)
            self.camThread.print_mess.connect(self.messPrint3)
            self.camThread.clearscene.connect(self.ClearScene2)
            # self.camThread.setbtn.connect(self.SetBtn1)
            self.camThread.endadj.connect(self.CAMAdjEnd1)
            self.camThread.pubtwist.connect(self.pubTwist1)
            

            self.camThread.start()

            self.mrMov_PB.setEnabled(True)

        else:
            self.scene2.clear()
            self.scene1.clear()
            
            bCAMAdapt = False
            self.camApply_PB.setText('CAM Apply')

            self.camThread.procExit()

            self.mrMov_PB.setDisabled(True)
            

    def onCalibrationBtn(self):
        global bRunCalib

        if bRunCalib is False:
            bRunCalib = True            

            self.calprocThread = Thread_CheckerBoard_CalibrationProc(self)

            self.calprocThread.VideoSignal1.connect(self.setImage1)
            
            self.calprocThread.printmess.connect(self.messPrint1)            
            # self.calprocThread.printedit.connect(self.printEdit2)            
            self.calprocThread.start()

            self.print_MessStr("[R]:Start calibration ...")

    # def CallHCamRGB(self, msgs):
    #     # try:

    #     self.cvImg = self.cvBridge.imgmsg_to_cv2(msgs, "bgr8")            

    #     self.rgbImg = cv2.cvtColor(self.cvImg, cv2.COLOR_BGR2RGB)

    #     h, w, ch = self.rgbImg.shape
    #     # h, w, ch = rotImg.shape
    #     bytesPerLine = ch * w
    #     print("h:" + str(h) + " w:" + str(w) + " ch:" + str(ch))

    #     # rotImg = cv2.rotate(rgbImg, cv2.ROTATE_90_CLOCKWISE)


    #     # flipImg = cv2.flip(rgbImg, 0)

    #     # cv2.imshow('flir cam', flipImg)
    #     # cv2.waitKey(1)


    #     self.qtImg = QImage(self.rgbImg.data, w, h, bytesPerLine, QImage.Format_RGB888)
    #     # qtImg = QImage(rotImg.data, w, h, bytesPerLine, QImage.Format_RGB888)
    #     # qtImg = QImage(flipImg.data, w, h, bytesPerLine, QImage.Format_RGB888)
    #     self.scaledImg = self.qtImg.scaled(301, 201, Qt.KeepAspectRatio)

    #     self.scene1.addPixmap(QPixmap.fromImage(self.scaledImg))
    #     self.camcap_graphicsView.setScene(self.scene1)

    #     # except:
    #     #     pass

    # def Callback_EStop(self, msg):
    #     self.bEStop_Status = msg.data
    #     #print("EStop Status : {}\r".format(self.bEStop_Status))
        
    #     if self.bEStop_Status != 1 and self.bEStop == False:
    #     # if self.bEStop_Status != 0 and self.bEStop == False:
    #         if self.bEStop_Status == 0:
    #             print("Detected Lidar Field\r")
    #         elif self.bEStop_Status == 3:
    #             print("Pushed E-Stop Button\r")
    #         elif self.bEStop_Status == 2:
    #             print("Detected Lidar Field and Pushed E-Stop Button\r")

    #         self.bprevEStop = self.bEStop
    #         self.bEStop = True

    def CallEStop(self, msgs):
        global logdata
    
        if self.bStartOK == False:
            return

        # self.bEstop = msgs.data
        
        self.bEStop_Status = msgs.data
        #print("EStop Status : {}\r".format(self.bEStop_Status))
        
        if self.bHDIRun == True:
            self.naviHDI_th.estopdata.emit(self.bEStop_Status)
        elif self.bCRDIRun == True:
            self.naviCRDI_th.estopdata.emit(self.bEStop_Status)
        
        if self.bEStop_Status != 1 and self.bEStop == False:
        # if self.bEStop_Status != 0 and self.bEStop == False:
            if self.bEStop_Status == 0:
                print("Detected Lidar Field\r")
                logdata["safety_info"] = 0x03
            elif self.bEStop_Status == 3:
                print("Pushed E-Stop Button\r")
                logdata["safety_info"] = 0x01
            elif self.bEStop_Status == 2:
                print("Detected Lidar Field and Pushed E-Stop Button\r")
                logdata["safety_info"] = 0x01

            self.bprevEStop = self.bEStop
            self.bEStop = True
            
        elif self.bEStop == True and self.bEStop_Status == 1:
            # elif self.bEStop == True and self.bEStop_Status == 0:
            print("Release E-stop/ Lidar detecting\r")
            
            self.bprevEStop = self.bEStop
            self.bEStop = False
    
        if self.bEStop is True:
            # logdata["safety_info"] = 0x01
            logdata["robot_status"] = "EStop"
            self.write_log()
            
            self.estopLED_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))

            self.mrstat_lineEdit.setText("!! ESTOP !!")

            self.bprevEStop = True

            if self.bRunNaviWP is True:
                self.onWPStopBtn()

            if self.bRunTrajNavi is True:
                self.onTrajStopBtn()

            if self.bCSParking is True:
            #     self.bCSParking = False
            #     cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            #     os.system(cmdstr)

                self.chargingLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

        else:
            if self.bprevEStop is True:
                self.bprevEStop = False
                
                logdata["safety_info"] = 0x00
            
                self.estopLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
                self.mrstat_lineEdit.setText("Release ESTOP")

    @pyqtSlot(float)
    def pubTwist1(self, xvel):
        twist = Twist()            
        twist.linear.x = xvel; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
        self.pub_twist.publish(twist)

        print("pubTwist1 :" + str(xvel))

    @pyqtSlot(float)
    def pubTwist2(self, xvel):
        twist = Twist()            
        twist.linear.x = xvel; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
        self.pub_twist.publish(twist)

        print("pubTwist2 :" + str(xvel))

    @pyqtSlot(float)
    def pubTwist3(self, xvel):
        twist = Twist()            
        twist.linear.x = xvel; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
        self.pub_twist.publish(twist)

        print("pubTwist3 :" + str(xvel))


    @pyqtSlot(int)
    def RunPackage(self, num):
        if num == 1:
            if self.bBringUpRun == False:
                self.onBringupBtn()
        elif num == 2:
            if self.b2LRFRun == False:
                self.on2LFRBtn()
        elif num == 3:
            if self.bOPRun == False:
                self.onOperationBtn()
        elif num == 4:
            if self.bNaviRun == False:
                self.onNavigationBtn()
        elif num == 5:
            if self.bRVIZRun == False:
                self.onRVIZBtn()
        elif num == 6:
            if self.bWPCRun == False:
                self.onWPCBtn()
                if self.bRunTrajNavi == True:
                    self.onTrajStartBtn()
        elif num == 7:
            if self.bDSRRobotRun == False:
                self.onDSRRobotBtn()

        elif num == 8:
            if self.bDSRTrajRun == False:
                self.onDSRTrajBtn()

            self.bReadyDone = True
            self.readyPB.setText('Init')
            self.print_MessStr("Done ready...")
            self.robot_mode = "ReadyDone"            

        elif num == 9:
            self.onInitPosition()

    @pyqtSlot(int)
    def ExitPackage(self, num):
        if num == 1:
            self.onBringupBtn()
        elif num == 2:
            self.on2LFRBtn()
        elif num == 3:
            self.onOperationBtn()
        elif num == 4:
            self.onNavigationBtn()
        elif num == 5:
            self.onRVIZBtn()
        elif num == 6:
            self.onWPCBtn()
            if self.bRunTrajNavi == True:
                self.onTrajStartBtn()
        elif num == 7:
            self.onDSRRobotBtn()
        elif num == 8:
            self.onDSRTrajBtn()
        
            self.bReadyDone = False
            self.readyPB.setText('Ready')
            self.print_MessStr("Exit all ready...")
            self.robot_mode = "None"
        

    @pyqtSlot(int)
    def SetRunState(self, val):
        self.ready_progBar.setValue(val)

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
            
            
    # def pubRobotcontrolStatus(self, stat):
    #     rcs = RobotControlStatus()
    #     rcs.status = stat
        
    #     if stat == RobotControlStatus.ERROR:
    #         rcs.status_description = "Robot OP Status : Error"
    #     elif stat == RobotControlStatus.IDLING:
    #         rcs.status_description = "Robot OP Status : Idling"
    #     elif stat == RobotControlStatus.NAVI:
    #         rcs.status_description = "Robot OP Status : Waypoint navigation"
    #     elif stat == RobotControlStatus.DSR:
    #         rcs.status_description = "Robot OP Status : Moving DSR robot"
    #     elif stat == RobotControlStatus.UV:
    #         rcs.status_description = "Robot OP Status : Using UV Lamp"
    #     elif stat == RobotControlStatus.SPARY:
    #         rcs.status_description = "Robot OP Status : Using Spary pump"
    #     elif stat == RobotControlStatus.UVSPARY:
    #         rcs.status_description = "Robot OP Status : Using Spary pump and UV Lamp"
    #     elif stat == RobotControlStatus.CHARGING:
    #         rcs.status_description = "Robot OP Status : Charging in progress"
            
    #     self.rcspub.publish(rcs) 


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

            # uint_val = UInt8()
            # uint_val.data = self.SelectWF
            # self.put_ignoreWF.publish(uint_val)
            print("IgnoreWarningField(FrontWF) :", self.SelectWF)

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

            # uint_val = UInt8()
            # uint_val.data = self.SelectWF
            # self.put_ignoreWF.publish(uint_val)
            print("IgnoreWarningField(FrontWF) :", self.SelectWF)

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

            # uint_val = UInt8()
            # uint_val.data = self.SelectWF
            # self.put_ignoreWF.publish(uint_val)
            print("IgnoreWarningField(RearWF) :", self.SelectWF)

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

            # uint_val = UInt8()
            # uint_val.data = self.SelectWF
            # self.put_ignoreWF.publish(uint_val)
            print("IgnoreWarningField(RearWF) :", self.SelectWF)

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

            # cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectCRDIWF) + "\" -1 "
            # print(cmdstr)
            # os.system(cmdstr)

            uint_val = UInt8()
            uint_val.data = self.SelectCRDIWF
            self.put_ignoreWF.publish(uint_val)
            print("IgnoreWarningField(CRDIFrontWF) :", self.SelectCRDIWF)

            self.print_MessStr("Activate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            #self.onFrontWFCtrlPBtn()
            #print("On Front WP")
        else:
            self.bCRDIFrontWF = False
            self.SelectCRDIWF = ((self.SelectCRDIWF & 0b11) & 0b10)
            
            print(self.SelectCRDIWF)

            self.frontwfcPB.setText("Disable Front WF")

            # cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectCRDIWF) + "\" -1 "
            # print(cmdstr)
            # os.system(cmdstr)

            uint_val = UInt8()
            uint_val.data = self.SelectCRDIWF
            self.put_ignoreWF.publish(uint_val)
            print("IgnoreWarningField(CRDIFrontWF) :", self.SelectCRDIWF)

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

            # cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectCRDIWF) + "\" -1 "
            # print(cmdstr)
            # os.system(cmdstr)

            uint_val = UInt8()
            uint_val.data = self.SelectCRDIWF
            self.put_ignoreWF.publish(uint_val)
            print("IgnoreWarningField(CRDIRearWF) :", self.SelectCRDIWF)

            self.print_MessStr("Activate warning field of rear lidar.")
            self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            #self.onRearWFCtrlPBtn()
            #print("On Rear WP")
        else:
            self.bCRDIRearWF = False
            self.SelectCRDIWF = ((self.SelectCRDIWF & 0b11) & 0b01)
            print(self.SelectCRDIWF)

            self.rearwfcPB.setText("Disable Rear WF")

            # cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectCRDIWF) + "\" -1 "
            # print(cmdstr)
            # os.system(cmdstr)

            uint_val = UInt8()
            uint_val.data = self.SelectCRDIWF
            self.put_ignoreWF.publish(uint_val)
            print("IgnoreWarningField(CRDIRearWF) :", self.SelectCRDIWF)

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
    

    def onROSCoreBtn(self):
        if self.bRoscoreRun == False:
            self.bRoscoreRun = True
            cmd = "python3 ~/bin/target_term -run 2 roscore"

            try:
                self.roscore_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run roscore process...")

            self.print_MessStr("Run roscore")
            self.runRoscore_PB.setText('Stop Roscore')

        else:
            self.bRoscoreRun = False

            try:            
                self.roscore_proc.kill()
            except:
                print("roscore process has died...")
            
            self.runRoscore_PB.setText('Run Roscore')
            self.print_MessStr("Stop roscore")

        # os.system("gnome-terminal -- roscore")
        # self.print_MessStr("Run roscore")


    def onBringupBtn(self):
        if self.bBringUpRun is False:  
            self.bBringUpRun = True      

            cmd = "python3 ~/bin/target_term -run 3 roslaunch zetabank_bringup zetabank_robot.launch"
            try:
                self.rosbringup_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run rosbringup process...")

            # os.system("python3 ~/bin/target_term -run 2 roslaunch zetabank_bringup zetabank_robot.launch")
            #os.system("gnome-terminal -- roslaunch zetabank_bringup zetabank_robot.launch")
            self.print_MessStr("Run roslaunch zetabank_robot.launch")
            self.run_bringupPB.setText('Stop Bringup')

            self.run_2LRFPB.setEnabled(True)
            self.run_teleopPB.setEnabled(True)
            # self.run_wpcPB.setEnabled(True)
            self.motionctrl_PB.setEnabled(True)
            self.run_OPPB.setEnabled(True)
            self.webmanager_PB.setEnabled(True)
            self.viewsonar_checkBox.setEnabled(True)

        else:
            self.bBringUpRun = False      

            self.run_2LRFPB.setDisabled(True)
            self.run_teleopPB.setDisabled(True)
            # self.run_wpcPB.setDisabled(True)
            self.motionctrl_PB.setDisabled(True)
            self.run_OPPB.setDisabled(True)
            self.webmanager_PB.setDisabled(True)
            self.viewsonar_checkBox.setDisabled(True)

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /STM")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /powerctrl")
            # sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zeta_mdrobot_motor_control_node")
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /robot_state_publisher")
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /sick_safetyscanners_front")
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /sick_safetyscanners_rear")
            sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /stm_starter")
            # sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zetabank_diagnostics")            
            sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /charging_act")

            # self.print_MessStr("Exit zetabank_robot.launch")
            # self.run_bringupPB.setText('Run Bringup')

            sleep(2.0)

            try:            
                self.rosbringup_proc.kill()
            except:
                print("ros bringup process has died...")

            self.print_MessStr("Exit zetabank_robot.launch")
            self.run_bringupPB.setText('Run Bringup')
        
    def on2LFRBtn(self):
        if self.b2LRFRun is False:
            self.b2LRFRun = True
            cmd = "python3 ~/bin/target_term -run 4 roslaunch ira_laser_tools laserscan_multi_merger.launch"
            try:
                self.twolrf_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run rosbringup process...")

            # os.system("python3 ~/bin/target_term -run 3 roslaunch ira_laser_tools laserscan_multi_merger.launch")
            self.print_MessStr("Run roslaunch laserscan_multi_merger.launch")
            self.run_2LRFPB.setText('Stop 2LRF')

        else:
            self.b2LRFRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /laserscan_multi_merger")      

            sleep(2.0)         

            try:            
                self.twolrf_proc.kill()
            except:
                print("lrfmultimege process has died...")   

            self.print_MessStr("Exit roslaunch laserscan_multi_merger.launch")
            self.run_2LRFPB.setText('Act 2LRF')

    def onOperationBtn(self):
        if self.bOPRun is False:  
            self.bOPRun = True    

            cmd = "python3 ~/bin/target_term -run 5 roslaunch zetabot_main operate_robot.launch"
            try:
                self.oprobot_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run oprobot process...")

            self.print_MessStr("Run roslaunch operater_robot.launch")

            sleep(0.5)

            cmd = "python3 ~/bin/target_term -run 6 roslaunch autocharge parkingcs.launch"
            try:
                self.autocharge_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run parkingcs process...")

            self.print_MessStr("Run roslaunch parkingcs.launch")

            self.run_OPPB.setText('Stop OP.')
            self.run_slamPB.setEnabled(True)
            # self.runCGSLAM_PB.setEnabled(True)
            self.run_naviPB.setEnabled(True)

        else:
            self.bOPRun = False            
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /initial_pos_srv")
            sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /battery_log")
            # sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /autocharge_act_srv")
            sleep(2.0)

            try:            
                self.oprobot_proc.kill()
            except:
                print("op robot process has died...")

            sleep(0.2)

            try:            
                self.autocharge_proc.kill()
            except:
                print("parkincs process has died...")

            sleep(1)

            self.run_slamPB.setDisabled(True)
            # self.runCGSLAM_PB.setDisabled(True)
            self.run_naviPB.setDisabled(True)

            self.print_MessStr("Exit operater_robot.launch")
            self.run_OPPB.setText('Run OP.')

    def onSlamBtn(self):        
        if self.bSLAMRun is False:
            self.bSLAMRun = True
            
            # os.system("python3 ~/bin/target_term -run 2 roslaunch zetabank_slam zetabank_slam.launch")

            cmd = "python3 ~/bin/target_term -run 7 roslaunch zetabank_slam zetabank_slam.launch"
            try:
                self.gridmap_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run gridmap process...")

            self.print_MessStr("Run roslaunch zetabank_slam.launch")
            self.run_slamPB.setText('Stop SLAM')
            self.run_slamrvizPB.setEnabled(True)
        else:
            self.bSLAMRun = False      

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zetabank_slam_gmapping")

            try:            
                self.gridmap_proc.kill()
            except:
                print("gridmap_proc process has died...")

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /zetabank_slam_gmapping")      

            self.print_MessStr("Exit roslaunch zetabank_slam.launch")
            self.run_slamPB.setText('Run SLAM')
            self.run_slamrvizPB.setDisabled(True)

    def onSLAMRVIZBtn(self):
        if self.bSLAMRVIZRun is False:
            self.bSLAMRVIZRun = True
            
            # cmdstr = "python3 ~/bin/target_term -run 13 rosrun rviz rviz -d " + slamrviz_fname
            # os.system(cmdstr)
            
            cmd = "python3 ~/bin/target_term -run 8 rosrun rviz rviz -d " + slamrviz_fname
            try:
                self.slamrviz_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run slamrviz process...")

            self.print_MessStr("Run rosrun SLAM rviz")
            self.run_slamrvizPB.setText('Stop SLAMRVIZ')

            self.savemapPB.setEnabled(True)
        else:
            self.bSLAMRVIZRun = False
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill $(rosnode list | grep rviz_*)")

            os.system("python3 ~/bin/target_term -run 10 rosnode kill $(rosnode list | grep rviz_*)")

            sleep(2.0)         

            try:            
                self.slamrviz_proc.kill()
            except:
                print("slamrviz process has died...")   

            self.print_MessStr("Stop rosrun SLAM rviz")
            self.run_slamrvizPB.setText('Run SLAMRVIZ')

            self.savemapPB.setDisabled(True)

    def onSaveMapBtn(self):
        if self.bSLAMRun == True:
        # if self.bSLAMRun == True or self.bCartographerSLAMRun == True:
        # os.system("python3 ~/bin/target_term -run 10 rosrun map_server map_saver -f dgcity_sp00")

            cmdstr = "python3 ~/bin/target_term -run 10 rosrun map_server map_saver -f " + self.mapfile_lineEdit.text()
            os.system(cmdstr)

            print(cmdstr)
            
            self.print_MessStr("Save map data...")
        else:
            self.print_MessStr("Gridmap Node or Cartographer node is not running!")

    def onNavigationBtn(self):
        if self.bNaviRun is False:
            self.bNaviRun = True

            # /os.system("python3 ~/bin/target_term -run 7 source ~/catkin_ws/devel/setup.bash")
            if self.bReadyDone == True:
                self.bReadyDone = False

            os.system("python3 ~/bin/target_term -run 10 source ~/catkin_ws/devel/setup.bash")

            sleep(2)

            cmd = "python3 ~/bin/target_term -run 9 roslaunch zetabank_navigation normal_navigation.launch"
            try:
                self.navigation_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run navigation process...")

            # os.system("python3 ~/bin/target_term -run 5 roslaunch zetabank_navigation normal_navigation.launch")
            self.print_MessStr("Run roslaunch normal_navigation.launch")
            self.run_naviPB.setText('Stop Navi.')
            self.run_rvizPB.setEnabled(True)
        else:
            self.bNaviRun = False        
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /amcl")       
            sleep(0.5)      
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /map_server")
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /move_base")            
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /AMCL_particles")            
            sleep(2.0)
            
            try:            
                self.navigation_proc.kill()
            except:
                print("navigation process has died...")   

            sleep(2.0)

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /amcl")             
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /map_server")
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /move_base")            
            self.print_MessStr("Exit roslaunch normal_navigation.launch")
            self.run_naviPB.setText('Run Navi.')
            self.run_rvizPB.setDisabled(True)

    def onRVIZBtn(self):
        if self.bRVIZRun is False:
            self.bRVIZRun = True

            if self.bNaviRun == True:
                self.bReadyDone = True
            
            cmd = "python3 ~/bin/target_term -run 11 rosrun rviz rviz -d " + navirviz_fname
            try:
                self.navirviz_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run navirviz process...")
            
            # cmdstr = "python3 ~/bin/target_term -run 6 rosrun rviz rviz -d " + navirviz_fname
            # os.system(cmdstr)
            
            self.print_MessStr("Run rosrun rviz")
            self.run_rvizPB.setText('Stop RVIZ')
            
            self.angle_lcdNumber.setEnabled(True)

            self.run_wpcPB.setEnabled(True)
            self.run_makeWPPB.setEnabled(True)
            self.csparking_PB.setEnabled(True)
            self.getamclpos_PB.setEnabled(True)
            self.chargingmode_checkBox.setEnabled(True)
            # self.initangle_PB.setEnabled(True)

            rospy.Subscriber("/heading_angle", Int16, self.recv_HeadingAngle)
            
        else:
            self.bRVIZRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill $(rosnode list | grep rviz_*)")

            sleep(2.0)         

            try:            
                self.navirviz_proc.kill()
            except:
                print("navirviz process has died...")     

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill $(rosnode list | grep rviz_*)")
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill 'rosnode list | grep rviz_*'")

            self.print_MessStr("Stop rosrun rviz")
            self.run_rvizPB.setText('Run RVIZ')
            
            self.angle_lcdNumber.setDisabled(True)

            self.run_wpcPB.setDisabled(True)
            self.run_makeWPPB.setDisabled(True)
            self.csparking_PB.setDisabled(True)
            self.getamclpos_PB.setDisabled(True)
            self.chargingmode_checkBox.setDisabled(True)

    def recv_HeadingAngle(self, msgs):
        self.angle_lcdNumber.display(msgs.data)

    def onWPCBtn(self):
        if self.bWPCRun is False:
            self.bWPCRun = True

            # os.system("python3 ~/bin/target_term -run 7 roslaunch navi_waypoint navigationWayPoint.launch")

            cmd = "python3 ~/bin/target_term -run 12 roslaunch navi_waypoint navigationWayPoint.launch"
            try:
                self.wpctrl_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run wpctrl process...")
            
            self.print_MessStr("Run roslaunch navigationWayPoint.launch")
            self.run_wpcPB.setText('Stop WPCtrl')      

            self.wp_CB.setEnabled(True)
            self.traj_CB.setEnabled(True)
            self.wp_startPB.setEnabled(True)
            self.wp_stopPB.setDisabled(True)
            self.traj_startPB.setEnabled(True)
            self.traj_stopPB.setDisabled(True)
            self.trajmode_checkBox.setEnabled(True)
            self.usesonar_checkBox.setEnabled(True)

            self.setCRDIFrontWF(False)
            self.setCRDIRearWF(False)

        else:
            self.bWPCRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /navigation_waypoints_node")      

            sleep(2.0)         

            try:            
                self.wpctrl_proc.kill()
            except:
                print("wpctrl process has died...")  

            self.print_MessStr("Stop roslaunch navigationWayPoint.launch")
            self.run_wpcPB.setText('Run WPCtrl')

            self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            self.wp_CB.setDisabled(True)
            self.traj_CB.setDisabled(True)
            self.wp_startPB.setDisabled(True)
            self.wp_stopPB.setDisabled(True)
            self.traj_startPB.setDisabled(True)
            self.traj_stopPB.setDisabled(True)
            self.trajmode_checkBox.setDisabled(True)
            self.usesonar_checkBox.setDisabled(True)

    def onActivatedwpCB(self, text):
        self.wpCBStr = text

    def onActivatedtrajCB(self, text):
        self.trajCBStr = text

    def onWPStartBtn(self):

        # 2022. 08. 10
        if self.bFrontWF == True:
            self.onFrontWFCtrlPBtn()

        if self.bRearWF == True:
            self.onRearWFCtrlPBtn()

        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = self.wpCBStr
        self.ncpub.publish(nc)

        self.wp_startPB.setDisabled(True)
        self.wp_stopPB.setEnabled(True)

        self.bRunNaviWP = True
        mstr = "[WP] Start way point : " + self.wpCBStr
        self.print_MessStr(mstr)
        
        logdata["robot_status"] = "Driving"

    def onWPStopBtn(self):
        # 2022. 08. 10
        if self.bFrontWF == False:
            self.onFrontWFCtrlPBtn()

        if self.bRearWF == False:
            self.onRearWFCtrlPBtn()

        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.wpCBStr
        self.ncpub.publish(nc)

        self.wp_startPB.setEnabled(True)
        self.wp_stopPB.setDisabled(True)

        self.bRunNaviWP = False
        mstr = "[WP] Stop way point : " + self.wpCBStr
        self.print_MessStr(mstr)
        
        logdata["robot_status"] = "None"

    def onTrajStartBtn(self):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = self.trajCBStr
        self.ncpub.publish(nc)

        self.traj_startPB.setDisabled(True)
        self.traj_stopPB.setEnabled(True)

        self.bRunTrajNavi = True
        mstr = "Start trajector : " + self.trajCBStr
        self.print_MessStr(mstr)
        
        logdata["robot_status"] = "Driving"
        
    def onTrajStopBtn(self):
        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.trajCBStr
        self.ncpub.publish(nc)

        self.traj_startPB.setEnabled(True)
        self.traj_stopPB.setDisabled(True)

        self.bRunTrajNavi = False
        mstr = "Stop trajector : " + self.trajCBStr
        self.print_MessStr(mstr)
        
        logdata["robot_status"] = "None"


    def onMakeWPBtn(self):
        if self.bMakeWPRun is False:
            self.bMakeWPRun = True

            # os.system("python3 ~/bin/target_term -run 8 roslaunch make_waypoint makewaypoint.launch")

            cmd = "python3 ~/bin/target_term -run 13 roslaunch make_waypoint makewaypoint.launch"
            try:
                self.makewaypoint_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run make waypoint process...")
            
            self.print_MessStr("Run roslaunch navigationWayPoint.launch")
            self.run_makeWPPB.setText('Stop makeWP')

            self.run_saveWPPB.setEnabled(True)
        else:
            self.bMakeWPRun = False

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /make_waypoint_node")                         
            try:            
                self.makewaypoint_proc.kill()
            except:
                print("make waypoint process has died...")   
            
            self.print_MessStr("Stop roslaunch navigationWayPoint.launch")
            self.run_makeWPPB.setText('Run makeWP')

            self.run_saveWPPB.setDisabled(True)

    def onSaveWPBtn(self):
        # os.system("python3 ~/bin/target_term -run 10 rostopic pub save_wp std_msgs/String \"save\" -1")
        
        cmdstr = "python3 ~/bin/target_term -run 10 rostopic pub -1 save_wp zetabank_msgs/SaveWaypoint -- 'save' '" + self.WPfile_lineEdit.text() + "'"
        os.system(cmdstr)

        print(cmdstr)

        self.print_MessStr("Run rostopic pub save_wp")

    def onParkingChargingStationBtn(self):
        if self.bCSParking == False:            
            self.bCSParking = True

            self.setCRDIFrontWF(False)
            self.setCRDIRearWF(False)

            if self.bWPCRun is True:
                self.onWPStopBtn()

                self.ClearCostMap()
                sleep(2)

            if self.bRunTrajNavi is True:
                self.onTrajStopBtn()

                self.ClearCostMap()
                sleep(2)

            sleep(1)
            
            self.client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            self.client.wait_for_server()

            self.setLaserAct(False)

            goal = ChargingGoal()
            print("goal : ")
            print(goal)
            self.client.send_goal(goal)

            self.print_MessStr("Send goal for parking charging station.")
            self.csparking_PB.setText('PCS Cancel')
            self.robot_mode = "Auto_ParkingCS"
        else:
            self.bCSParking = False
            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)

            print(cmdstr)

            self.chargingLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

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

            self.robot_mode = "Navigation_Normal"

            sleep(1)

            self.onInitPosition()

            sleep(2)

            self.setLaserAct(True)
            
            self.bHDIAutoPark = False

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

    def InitMCValueBtn(self):
        global mctrl_th

        mctrl_th.initVal()
        
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
 
   
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def onTeleopBtn(self):
        if self.bTeleOPRun is False:
            self.bTeleOPRun = True
            
            # os.system("python3 ~/bin/target_term -run 3 roslaunch teleop_keyandjoy zetabank_teleop_key.launch")

            cmd = "python3 ~/bin/target_term -run 14 roslaunch teleop_keyandjoy zetabank_teleop_key.launch"
            try:
                self.teleop_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run teleop process...")

            self.print_MessStr("Run roslaunch zetabank_teleop_key.launch")
            self.run_teleopPB.setText('Stop teleop')
        else:
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /teleop_key")                        
            self.print_MessStr("Exit roslaunch zetabank_teleop_key.launch")
            self.run_teleopPB.setText('Run teleop')

    
    def onDSRRobotBtn(self):
        if self.bDSRRobotRun is False:
            self.bDSRRobotRun = True
            
            # os.system("python3 ~/bin/target_term -run 9 roslaunch zetabank_dsr zetabank_dsr.launch mode:=real host:=192.168.11.110 port:=12345 model:=m0609 color:=white")
            
            cmd = "python3 ~/bin/target_term -run 15 roslaunch zetabank_dsr zetabank_dsr.launch mode:=real host:=192.168.11.110 port:=12345 model:=m0609 color:=white"
            try:
                self.dsrrobot_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run dsr robot process...")

            self.run_dsrRobotPB.setText('Stop DSR')
            self.print_MessStr("Run roslaunch zetabank_dsr.launch")

            # self.initpos_dsrRobotPB.setEnabled(True)
            self.run_dsrTrajPB.setEnabled(True)
            
        else:
            self.bDSRRobotRun = False

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01/controller_spawner")
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01/robot_state_publisher")
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01/spawn_create_model")
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01/world_tf")
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01m0609")
            sleep(0.5)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr01m0609/controller_spawner")
            sleep(1.0)

            try:            
                self.dsrrobot_proc.kill()
            except:
                print("dsr robot process has died...")   

            # os.system("python3 ~/bin/target_term -run 9 kill -2 $$")

            self.run_dsrRobotPB.setText('Run DSR')            
            self.print_MessStr("Stop roslaunch zetabank_dsr.launch")

            # self.initpos_dsrRobotPB.setDisabled(True)
            self.run_dsrTrajPB.setDisabled(True)
            self.zeropos_dsrRobotPB.setDisabled(True)
            self.dsrtraj_CB.setEnabled(True)
            self.dsrtraj_runPB.setDisabled(True)
            self.dsrtraj_stopstartPB.setDisabled(True)

    def onDSRTrajBtn(self):
        if self.bDSRTrajRun is False:
            self.bDSRTrajRun = True

            # os.system("python3 ~/bin/target_term -run 11 roslaunch zetabank_dsr zetabank_dsr_control.launch")
            
            cmd = "python3 ~/bin/target_term -run 16 roslaunch zetabank_dsr zetabank_dsr_control.launch"
            try:
                self.dsrtraj_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run dsr trajectory process...")

            self.print_MessStr("Run roslaunch zetabank_dsr_control.launch")
            self.run_dsrTrajPB.setText('Stop DSR Traj')

            self.initpos_dsrRobotPB.setEnabled(True)
            self.zeropos_dsrRobotPB.setEnabled(True)
            self.dsrtraj_CB.setEnabled(True)
            self.dsrtraj_runPB.setEnabled(True)
            self.dsrtraj_stopstartPB.setEnabled(True)
            self.camApply_PB.setEnabled(True)
            # self.mrMov_PB.setEnabled(True)

        else:
            self.bDSRTrajRun = False

            zbdsr_traj = ZBDSRSetTraj()
            zbdsr_traj.trajnum = self.cur_trajnum
            zbdsr_traj.setstatus = ZBDSRSetTraj.EXIT
            self.dsrpub.publish(zbdsr_traj)

            sleep(0.5)

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /dsr_control")

            sleep(1.0)

            # os.system("python3 ~/bin/target_term -run 11 kill -2 $$")

            try:            
                self.dsrtraj_proc.kill()
            except:
                print("dsr trajectory process has died...") 

            self.print_MessStr("Stop roslaunch zetabank_dsr_control.launch")
            self.run_dsrTrajPB.setText('Run DSR Traj')            

            self.initpos_dsrRobotPB.setDisabled(True)
            self.zeropos_dsrRobotPB.setDisabled(True)
            self.dsrtraj_CB.setDisabled(True)
            self.dsrtraj_runPB.setDisabled(True)
            self.dsrtraj_stopstartPB.setDisabled(True)
            self.camApply_PB.setDisabled(True)
            self.mrMov_PB.setDisabled(True)

    def onActivateddsrtrajCB(self, text):
        self.dsrtrajCBStr = text

    def onInitPosDSRRobotBtn(self):
        zbdsr_traj = ZBDSRSetTraj()
        zbdsr_traj.trajnum = 100
        zbdsr_traj.setstatus = ZBDSRSetTraj.START
        self.dsrpub.publish(zbdsr_traj)

        mstr = "Initialize position of the DSR Robot."
        self.print_MessStr(mstr)

        # self.run_dsrTrajPB.setEnabled(True)
        # self.zeropos_dsrRobotPB.setEnabled(True)
        
        
    def onZeroPosDSRRobotBtn(self):
        zbdsr_traj = ZBDSRSetTraj()
        zbdsr_traj.trajnum = 101
        zbdsr_traj.setstatus = ZBDSRSetTraj.START
        self.dsrpub.publish(zbdsr_traj)

        mstr = "Zero position of the DSR Robot."
        self.print_MessStr(mstr)

    def changeTrajMode(self, state):
        if state == Qt.Checked:
            self.trajmode = NavigationControl.LOOP
            self.print_MessStr("Navi Waypoint Mode : Loop")
        else:
            self.trajmode = NavigationControl.GOAL
            self.print_MessStr("Navi Waypoint Mode : Goal")

    def onDSRTrajRunBtn(self):

        zbdsr_traj = ZBDSRSetTraj()
        val = re.findall(r'\d+', self.dsrtrajCBStr)
        val2 = map(int, val)

        self.cur_trajnum = val2[0]
        zbdsr_traj.trajnum = self.cur_trajnum
        zbdsr_traj.setstatus = ZBDSRSetTraj.START

        self.dsrpub.publish(zbdsr_traj)

        self.pubRobotcontrolStatus(RobotControlStatus.DSR)

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

    def ClearCostMap(self):
        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

    def CallbackRunTrajStatus(self, request):

        print(request)

        stat_str = "none"
        
        if self.bHDIRun == True:
            self.naviHDI_th.dsrtraj_runstat.emit(request.curstatus, request.trajnum)
        elif self.bCRDIRun == True:
            self.naviCRDI_th.dsrtraj_runstat.emit(request.curstatus, request.trajnum)
        
        if request.curstatus == ZBDSRTrajStatus.IDLING:
            stat_str = "Idling"
        elif request.curstatus == ZBDSRTrajStatus.RUNNING:
            stat_str = "Running"
        elif request.curstatus == ZBDSRTrajStatus.PAUSED:
            stat_str = "Paused"
        elif request.curstatus == ZBDSRTrajStatus.COMPLETED:
            stat_str = "Completed"
            # self.setCRDIFrontWF(False)
            # self.setCRDIRearWF(False)
        elif request.curstatus == ZBDSRTrajStatus.CANCELLED:
            stat_str = "Canceled"
        elif request.curstatus == ZBDSRTrajStatus.START:
            stat_str = "Start"
            # self.setCRDIFrontWF(True)
            # self.setCRDIRearWF(True)            
        elif request.curstatus == ZBDSRTrajStatus.STOP:
            stat_str = "Stop"
            # self.setCRDIFrontWF(False)
            # self.setCRDIRearWF(False)
        elif request.curstatus == ZBDSRTrajStatus.ESTOP:
            stat_str = "EStop"
            # self.setCRDIFrontWF(False)
            # self.setCRDIRearWF(False)
        elif request.curstatus == ZBDSRTrajStatus.RESTART:
            stat_str = "Restart"

        mstr = "TrajNum(" + str(request.trajnum) + "), Status:" + stat_str

        print(mstr)
        
        self.dsrstat_lineEdit.setText(mstr)

    def CallbackRunNaviCtrlStatus(self, request):
        # self.naviStatus = request.status
        if self.bStartOK == False:
            return
        
        if self.bHDIRun == True:
            self.naviHDI_th.navi_runstat.emit(request.status, request.status_description)
        elif self.bCRDIRun == True:
            self.naviCRDI_th.navi_runstat.emit(request.status, request.status_description)
        
        if self.bWPCRun is True:
            self.naviStatus = request.status
            if self.bEStop is False:
                mstr = request.status_description
                self.mrstat_lineEdit.setText(mstr)

            if self.naviStatus is NavigationControlStatus.ABORTED or self.naviStatus is NavigationControlStatus.ERROR or self.naviStatus is NavigationControlStatus.ERRGTGF:
              self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))
            elif self.naviStatus is NavigationControlStatus.RUNNING:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_BLUE_LED))
            elif self.naviStatus is NavigationControlStatus.COMPLETED:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
                if self.bRunNaviWP is True:
                    self.bRunNaviWP = False

                    # sleep(1)                                    

                    self.robot_mode = "None"                    

                    # 2022. 08. 10
                    if self.bFrontWF == True:
                        self.onFrontWFCtrlPBtn()
                        print("CallbackRunNaviCtrlStatus : onFrontWFCtrlPBtn")
                        sleep(1)

                    if self.bRearWF == True:
                        self.onRearWFCtrlPBtn()
                        print("CallbackRunNaviCtrlStatus : onRearWFCtrlPBtn")
                        sleep(1)

                    self.wp_startPB.setEnabled(True)
                    self.wp_stopPB.setDisabled(True)

            elif self.naviStatus is NavigationControlStatus.TRAJCOMPLETED:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

                if self.bRunTrajNavi is True:
                    self.bRunTrajNavi = False

                    self.traj_startPB.setEnabled(True)
                    self.traj_stopPB.setDisabled(True)

                    self.robot_mode = "None"

            elif self.naviStatus is NavigationControlStatus.CANCELLED or self.naviStatus is NavigationControlStatus.WARNNRGTO:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_YELLOW_LED))
            elif self.naviStatus is NavigationControlStatus.IDLING:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
            elif self.naviStatus is NavigationControlStatus.REAROBSTACLE:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_ORANGE_LED))
            else:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

        mstr = request.status_description
        self.mrstat_lineEdit.setText(mstr)

    def CallbackCSStatus(self, msgs):
        if self.bStartOK == False:
            return
        
        # if self.bCSParking is True and (self.robot_mode != "Charging" or self.robot_mode != "FullCharging"):
        #     self.csStatus = msgs.data
        #     mstr = "R<->C.S : " + self.csStatus
        #     self.mrstat_lineEdit.setText(mstr)

        #     if(self.csStatus == "contact"):
        #         self.chargingLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

        #         self.robot_mode = "Charging"
        #         # print("robot mode : %s" % (self.robot_mode))
        
        if self.bHDIRun == True:
            self.naviHDI_th.csstatus.emit(msgs.data)
        elif self.bCRDIRun == True:
            self.naviCRDI_th.csstatus.emit(msgs.data)

        if self.bCSParking is True and (self.robot_mode != "Charging" or self.robot_mode != "FullCharging"):
            self.csStatus = msgs.data
            mstr = "R<->C.S : " + self.csStatus
            self.mrstat_lineEdit.setText(mstr)

            if(self.csStatus == "contact"):
                self.chargingLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

                self.robot_mode = "Charging"
                # print("robot mode : %s" % (self.robot_mode))

                if self.bHDIRun == True:
                    self.bHDIRun = False
                    self.print_MessStr("Stop navi Hallway Disinfection Process thread.")
                    self.runhdiPB.setText("Run HDI")


        # if self.bCSParking == True:
        #     self.csStatus = request.data

        #     mstr = "Charging Station Status : " + self.csStatus

        #     self.mrstat_lineEdit.setText(mstr)

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


    def CallSetCAMView(self, msg):
        setVal = msg.data

        if setVal == True:
            self.bLauchCAMTopic = True

            cmd = "python3 ~/bin/target_term -run 19 roslaunch usb_cam usb_cam.launch"
            try:
                self.launchcamtopic_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

                self.print_MessStr("Launch cam live topic...")
            except:
                print("Failed to run navigation process...")

            self.camtopic_PB.setText('Stop CAM Topic')

        else:
            self.bLauchCAMTopic = False

            # sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam")     
            
            sleep(2.0)
            
            try:            
                self.launchcamtopic_proc.kill()
            except:
                print("usb cam nodes have died...") 

            self.print_MessStr("Stop cam live topic...")
            self.camtopic_PB.setText('Launch CAM Topic')
            
    def CallCurPoseInfo(self, msgs):
        global logdata
    
        if self.bGetCurPos == True:
            self.position_x = msgs.position.x
            self.position_y = msgs.position.y
            self.position_z = msgs.position.z

        else:
            pass
            
        logdata["pos_x"] = msgs.position.x
        logdata["pos_y"] = msgs.position.y
        logdata["pos_z"] = msgs.position.z
                    
    def CallbackCSStatusNUC(self, msg):
        if self.bStartOK == False:
            return

        csnuc_state = msg.data
        
        # if self.bLowbatPark == True and csnuc_state == 5:
        #     self.bLowbatPark = False            

        if csnuc_state == 6:
            self.bBatteryFull = True

            if self.robot_mode == "Charging":
                self.robot_mode = "FullCharging"
                print("Change robot mode to FullCharging")

        else:
            self.bBatteryFull = False
            
            
    def recv_Sonar(self, msgs):
        if self.bStartOK == False:
            return

        if self.bViewSonarData == True:

            self.VSDCnt += 1

            if self.VSDCnt >= 25:
                self.VSDCnt = 0
            
                rimu = Imu()
                rimu = msgs

                self.u1_progressBar.setValue(rimu.data[6])
                self.u2_progressBar.setValue(rimu.data[7])
                self.l1_progressBar.setValue(rimu.data[5])
                self.l2_progressBar.setValue(rimu.data[4])
                self.b1_progressBar.setValue(rimu.data[2])
                self.b2_progressBar.setValue(rimu.data[3])
                self.r1_progressBar.setValue(rimu.data[0])
                self.r2_progressBar.setValue(rimu.data[1])

    def changeUseCAMAdj(self, state):
        global bUseCAMAdj

        if state == Qt.Checked:
            bUseCAMAdj = True
        else:
            bUseCAMAdj = False


    def changeViewSD(self, state):
        self.VSDCnt = 0

        if state == Qt.Checked:
            self.bViewSonarData = True
        else:
            self.bViewSonarData = False

            self.u1_progressBar.setValue(0.0)
            self.u2_progressBar.setValue(0.0)
            self.l1_progressBar.setValue(0.0)
            self.l2_progressBar.setValue(0.0)
            self.b1_progressBar.setValue(0.0)
            self.b2_progressBar.setValue(0.0)
            self.r1_progressBar.setValue(0.0)
            self.r2_progressBar.setValue(0.0)

    def onInitPosition(self):

        start_pose_param = rospy.get_param("start_pose")

        publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.loginfo("This node sets the robot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
        start_pos = PoseWithCovarianceStamped()
        #filling header with relevant information
        start_pos.header.frame_id = "map"
        start_pos.header.stamp = rospy.Time.now()
        #filling payload with relevant information gathered from subscribing
        # to initialpose topic published by RVIZ via rostopic echo initialpose
        start_pos.pose.pose.position.x = start_pose_param['position_x']
        start_pos.pose.pose.position.y = start_pose_param['position_y']
        start_pos.pose.pose.position.z = 1.0

        start_pos.pose.pose.orientation.x = 0.0
        start_pos.pose.pose.orientation.y = 0.0
        start_pos.pose.pose.orientation.z = start_pose_param['orientation_z']
        start_pos.pose.pose.orientation.w = start_pose_param['orientation_w']

        start_pos.pose.covariance[0] = 0.25
        start_pos.pose.covariance[7] = 0.25
        start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[35] = 0.06853891945200942

        publisher.publish(start_pos)

        sleep(2)

        publisher.publish(start_pos)

        self.print_MessStr("Set the start position...")


    def onWebManagerLaunch(self):
        if self.bRunWebManager is False:

            cmd1 = "python3 ~/bin/target_term -run 17 roslaunch rosboard rosboard.launch"
            try:
                self.rosboard_proc = subprocess.Popen(args=cmd1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

                self.print_MessStr("Run roslaunch rosboard.launch")

                sleep(5)

                cmd2 = "python3 ~/bin/target_term -run 18 rosrun rosboard rosboard_node"
                try:
                    self.rosboardnode_proc = subprocess.Popen(args=cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

                    sleep(3)

                    self.print_MessStr("Run rosrun rosboard_node")
                    self.webmanager_PB.setText('Stop Web Manager')
        
                    self.bRunWebManager = True

                except:
                    print("Failed to launch rosboard_node...")
                    self.bRunWebManager = False
            except:
                print("Failed to launch rosboard...")
                self.bRunWebManager = False

        else:
            self.bRunWebManager = False

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /rosbridge_websocket")       
            sleep(1.0)        

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /web_video_server")       
            sleep(1.0)        

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /ros_api")       
            sleep(1.0)        

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /ros_pose_publisher")       
            sleep(2.0)        

            try:            
                self.rosboard_proc.kill()

                self.print_MessStr("Exit roslaunch rosboard.launch")
            except:
                print("rosboard_proc process has died...") 


            os.system("python3 ~/bin/target_term -run 10 rosnode kill /rosboard_node")       

            sleep(2.0)        

            try:            
                self.rosboardnode_proc.kill()
                self.print_MessStr("Exit rosrun rosboard_node")

            except:
                print("rosboardnode_proc process has died...") 

            self.webmanager_PB.setText('Run Web Manager')

    def onCAMViewBtn(self):
        if self.bCSParking == True:
            self.print_MessStr("Now the robot is doing auto parking.")
        else:
            if self.bCAMView == False:
                self.bCAMView = True
                
                self.online_webcams = QCameraInfo.availableCameras()

                if self.online_webcams:   
                    self.exist = QCameraViewfinder()
                    self.exist.show()

                for cam in self.online_webcams:
                    # print(cam.deviceName())
                    if cam.deviceName() == "/dev/video0":
                        self.webcam = QCamera(cam)
                        self.webcam.setViewfinder(self.exist)
                        self.webcam.setCaptureMode(QCamera.CaptureStillImage)
                        self.webcam.error.connect(lambda: self.alert(self.my_webcam.errorString()))
                        self.webcam.start()

                        self.print_MessStr("run camera viewwer...")

                self.camView_PB.setText('Stop View')

            else:
                self.bCAMView = False

                self.webcam.stop()

                self.exist.hide()

                self.print_MessStr("stop camera viewwer...")

                self.camView_PB.setText('CAM View')
       
        
    def onLaunchCAMTopic(self):

        if self.bLauchCAMTopic == False:
            self.bLauchCAMTopic = True

            cmd = "python3 ~/bin/target_term -run 20 roslaunch usb_cam usb_cam.launch"
            try:
                self.launchcamtopic_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

                self.print_MessStr("Launch cam live topic...")
            except:
                print("Failed to run navigation process...")

            self.camtopic_PB.setText('Stop CAM Topic')

        else:
            self.bLauchCAMTopic = False

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam")       
  
            sleep(2.0)
            
            try:            
                self.launchcamtopic_proc.kill()
            except:
                print("usb cam nodes have died...") 

            self.print_MessStr("Stop cam live topic...")
            self.camtopic_PB.setText('Launch CAM Topic')

    def onGetAMCLPosBtn(self):
        dialog = GetAMCLPosDialog()
        dialog.exec_()

    def onInitPosition(self):

        start_pose_param = rospy.get_param("start_pose")

        publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.loginfo("This node sets the robot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
        start_pos = PoseWithCovarianceStamped()
        #filling header with relevant information
        start_pos.header.frame_id = "map"
        start_pos.header.stamp = rospy.Time.now()
        #filling payload with relevant information gathered from subscribing
        # to initialpose topic published by RVIZ via rostopic echo initialpose
        start_pos.pose.pose.position.x = start_pose_param['position_x']
        start_pos.pose.pose.position.y = start_pose_param['position_y']
        start_pos.pose.pose.position.z = 1.0

        start_pos.pose.pose.orientation.x = 0.0
        start_pos.pose.pose.orientation.y = 0.0
        start_pos.pose.pose.orientation.z = start_pose_param['orientation_z']
        start_pos.pose.pose.orientation.w = start_pose_param['orientation_w']

        start_pos.pose.covariance[0] = 0.25
        start_pos.pose.covariance[7] = 0.25
        start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[35] = 0.06853891945200942

        publisher.publish(start_pos)

        sleep(2)

        publisher.publish(start_pos)

        self.print_MessStr("Set the start position...")


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
            # self.bCRDIRun = True
        
            #self.naviCRDI_th = Thread_ConfRoomDisfection(self)

            self.naviCRDI_th = Thread_ConfRoomDisfectionProcess(self)

            self.naviCRDI_th.setRunFlag(True)

            self.naviCRDI_th.navi_runstatstr.connect(self.crdi_naviRunState)
            self.naviCRDI_th.dsrtraj_runstatstr.connect(self.crdi_trajRunState)            
            # self.naviCRDI_th.navi_runstat.connect(self.crdi_naviRunState)
            # self.naviCRDI_th.dsrtraj_runstat.connect(self.crdi_trajRunState)
            self.naviCRDI_th.setrunflag.connect(self.setCSDIRunFlag)
            self.naviCRDI_th.setapflag.connect(self.setCSDIAutoParkingFlag)

            self.naviCRDI_th.setfwf.connect(self.setCRDIFrontWF)
            self.naviCRDI_th.setbwf.connect(self.setCRDIRearWF)
            self.naviCRDI_th.setuvlamp.connect(self.setUVLamp)
            self.naviCRDI_th.setuvblamp.connect(self.setBtUVLamp)
            # self.naviCRDI_th.setbtuvclamp.connect(self.setBtUVLamp)
            
            self.naviCRDI_th.setpcsbtn.connect(self.setPCSButton)
            self.naviCRDI_th.setlaser.connect(self.setLaserAct)

            self.naviCRDI_th.setpump.connect(self.setPump)
            self.naviCRDI_th.setsol.connect(self.setSol)

            # self.naviCRDI_th.pubtwist.connect(self.pubTwist3)
            
            self.naviCRDI_th.start()           

            self.print_MessStr("Run navi Conference Room Disinfection Process thread.")

            self.runcrdiPB.setText("Stop CRDI")

            self.bCRDIRun = True

        else:
            self.bCRDIRun = False

            self.naviCRDI_th.setRunFlag(False)

            self.print_MessStr("Stop navi Conference Room Disinfection Process thread.")

            self.runcrdiPB.setText("Run CRDI")

        
        
    def onHDIBtn(self):

        if self.bHDIRun == False:

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

            # 2022. 08. 10
            if self.bFrontWF == True:
                self.onFrontWFCtrlPBtn()

            if self.bRearWF == True:
                self.onRearWFCtrlPBtn()
                
        # if self.bHDIRun == False:
            self.naviHDI_th = Thread_HallowDisfectionProcess(self)
            
            self.naviHDI_th.setRunFlag(True)            

            self.naviHDI_th.navi_runstatstr.connect(self.hdi_naviRunState)
            self.naviHDI_th.dsrtraj_runstatstr.connect(self.hdi_trajRunState)
            self.naviHDI_th.setrunflag.connect(self.setHDIRunFlag)
            self.naviHDI_th.setapflag.connect(self.setHDIAutoParkingFlag)

            self.naviHDI_th.setfwf.connect(self.setFrontWF)
            self.naviHDI_th.setbwf.connect(self.setRearWF)
            self.naviHDI_th.setuvlamp.connect(self.setUVLamp)
            self.naviHDI_th.setuvblamp.connect(self.setBtUVLamp)

            self.naviHDI_th.setlaser.connect(self.setLaserAct)            
            self.naviHDI_th.setpcsbtn.connect(self.setPCSButton)

            self.scene1 = QGraphicsScene()
            self.scene2 = QGraphicsScene()   

            self.naviHDI_th.VideoSignal2.connect(self.setImage2)
            self.naviHDI_th.VideoSignal3.connect(self.setImage3)
            self.naviHDI_th.print_mess.connect(self.messPrint3)
            self.naviHDI_th.clearscene.connect(self.ClearScene)              
            self.naviHDI_th.endadj.connect(self.CAMAdjEnd2)       

            self.naviHDI_th.pubtwist.connect(self.pubTwist2)     
            # self.naviHDI_th.setmovbtn.connect(self.setMoveBtn)     
                                
            self.naviHDI_th.start()
            

            # self.naviHDI_th.getcurtraj.connect(self.cur_trajnum)            
                        
            self.print_MessStr("Run navi Hallway Disinfection Process thread.")

            self.runhdiPB.setText("Stop HDI")

            self.bHDIRun = True


        else:
            self.bHDIRun = False

            self.naviHDI_th.SetRunFlag(False)
            # self.naviHDI_th.SetCAMAdapt(False)

            self.print_MessStr("Stop navi Hallway Disinfection Process thread.")

            self.runhdiPB.setText("Run HDI")

    def onReadyBtn(self):

        if self.bReadyDone == False:
            self.bReadyDone = True

            self.ready_progBar.setValue(0)

            ready_th = Thread_RunReady(self)
            ready_th.runpackage.connect(self.RunPackage)
            ready_th.exitpackage.connect(self.ExitPackage)
            ready_th.setrunstate.connect(self.SetRunState)
            ready_th.setType(1)
            ready_th.start()

        else:
            self.bReadyDone = False
            self.ready_progBar.setValue(0)

            ready_th = Thread_RunReady(self)
            ready_th.runpackage.connect(self.RunPackage)
            ready_th.exitpackage.connect(self.ExitPackage)
            ready_th.setrunstate.connect(self.SetRunState)
            ready_th.setType(2)
            ready_th.start()

    # def onReadyBtn(self):
    #     if self.bReadyRun == False:
    #         self.bReadyRun = True

    #         self.onDSRRobotBtn()
    #         #sleep(5)

    #         self.onBringupBtn()
    #         sleep(6)
            
    #         self.onTraj1Btn()
    #         sleep(5)

    #         self.on2LFRBtn()
    #         #sleep(2)

    #         self.onInitPosDSRRobotBtn()
    #         sleep(5)

    #         self.onNavigationBtn()
    #         sleep(6)
            
    #         self.onRVIZBtn()
    #         sleep(5)
            
    #         self.onWPCBtn()
    #         sleep(5)
          
    #         self.readyPB.setText('Init')

    #         self.print_MessStr("Ready to initialize...")

    #     else:
    #         self.bReadyRun = False

    #         self.onInitPosDSRRobotBtn()
    #         sleep(3)

    #         self.onWPCBtn()
    #         sleep(3)

    #         self.onTraj1Btn()
    #         sleep(3)

    #         self.on2LFRBtn()
    #         sleep(3)

    #         self.onNavigationBtn()
    #         sleep(3)

    #         self.onDSRRobotBtn()
    #         sleep(3)

    #         self.onBringupBtn()
    #         sleep(8)

    #         self.onRVIZBtn()
    #         sleep(3)

    #         self.readyPB.setText('Ready')

    #         self.print_MessStr("Initialize...")
    
    def print_MessStr(self, str):
        self.messLV.appendRow(QStandardItem(str))
        self.listView.setModel(self.messLV)
        self.listView.scrollToBottom()

    
# ==================== I/O Control ====================
    def onWarningFieldDisablePBtn(self):
        # os.system("gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: 3\" ")
        uint_val = UInt8()
        uint_val.data = 3
        self.put_ignoreWF.publish(uint_val)
        print("IgnoreWarningField(CRDIFrontWF) :", 3)

        self.print_MessStr("Disable detecting warning field.")

    def onWarningFieldEnablePBtn(self):
        # os.system("gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: 0\" ")
        uint_val = UInt8()
        uint_val.data = 0
        self.put_ignoreWF.publish(uint_val)
        print("IgnoreWarningField(CRDIFrontWF) :", 0)        
        self.print_MessStr("Enable detecting warning field.")

    def onFrontWFCtrlPBtn(self):
        if self.bFrontWF is False:
            self.bFrontWF = True
            self.SelectWF = ((self.SelectWF & 0b11) | 0b01)
            print(self.SelectWF)

            self.frontwfcPB.setText("Enable Front WF")

            
            # cmdstr = "python3 ~/bin/target_term -run 10 rostopic pub /IgnoreWarningField std_msgs/UInt8 \'data: " + str(self.SelectWF) + "\' -1 "
            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)

            # uint_val = UInt8()
            # uint_val.data = self.SelectWF
            # self.put_ignoreWF.publish(uint_val)
            # print("IgnoreWarningField(FrontWF) :", self.SelectWF)

            self.print_MessStr("Activate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        else:
            self.bFrontWF = False
            self.SelectWF = ((self.SelectWF & 0b11) & 0b10)
            print(self.SelectWF)

            self.frontwfcPB.setText("Disable Front WF")

            # cmdstr = "python3 ~/bin/target_term -run 10 rostopic pub /IgnoreWarningField std_msgs/UInt8 'data: " + str(self.SelectWF) + "' -1 "
            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)

            # uint_val = UInt8()
            # uint_val.data = self.SelectWF
            # self.put_ignoreWF.publish(uint_val)
            # print("IgnoreWarningField(FrontWF) :", self.SelectWF)

            self.print_MessStr("Deactivate warning field of front lidar.")
            self.fwtled_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

    def onRearWFCtrlPBtn(self):
        if self.bRearWF is False:
            self.bRearWF = True
            self.SelectWF = ((self.SelectWF & 0b11) | 0b10)
            print(self.SelectWF)

            self.rearwfcPB.setText("Enable Rear WF")

            # cmdstr = "python3 ~/bin/target_term -run 10 rostopic pub /IgnoreWarningField std_msgs/UInt8 'data: " + str(self.SelectWF) + "' -1 "
            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)

            # uint_val = UInt8()
            # uint_val.data = self.SelectWF
            # self.put_ignoreWF.publish(uint_val)
            # print("IgnoreWarningField(RearWF) :", self.SelectWF)

            self.print_MessStr("Activate warning field of rear lidar.")
            self.rwtled_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        else:
            self.bRearWF = False
            self.SelectWF = ((self.SelectWF & 0b11) & 0b01)
            print(self.SelectWF)

            self.rearwfcPB.setText("Disable Rear WF")

            # cmdstr = "python3 ~/bin/target_term -run 10 rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            cmdstr = "gnome-terminal -- rostopic pub /IgnoreWarningField std_msgs/UInt8 \"data: " + str(self.SelectWF) + "\" -1 "
            print(cmdstr)
            os.system(cmdstr)
            
            # uint_val = UInt8()
            # uint_val.data = self.SelectWF
            # self.put_ignoreWF.publish(uint_val)
            # print("IgnoreWarningField(RearWF) :", self.SelectWF)

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

                # uint_val = UInt8()
                # uint_val.data = self.SelWFRange
                # self.put_warningFS.publish(uint_val)
                # print("WarningFieldSelect(FrontSWFR) :", self.SelWFRange)

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

                # uint_val = UInt8()
                # uint_val.data = self.SelWFRange
                # self.put_warningFS.publish(uint_val)
                # print("WarningFieldSelect(FrontSWFR) :", self.SelWFRange)

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

                # uint_val = UInt8()
                # uint_val.data = self.SelWFRange
                # self.put_warningFS.publish(uint_val)
                # print("WarningFieldSelect(RearSWFR) :", self.SelWFRange)

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

                # uint_val = UInt8()
                # uint_val.data = self.SelWFRange
                # self.put_warningFS.publish(uint_val)
                # print("WarningFieldSelect(RearSWFR) :", self.SelWFRange)

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

    # def onCoonectSerial1Btn(self):
    #     os.system("gnome-terminal -- rosrun rosserial_python serial_node.py _baud:=115200 _port:=\"/dev/ttyACM0\" __name:=\"imu\" ")
    #     self.print_MessStr("Connect to the safety Lidar serial comm.(ttyACM0)")

    # def onCoonectSerial2Btn(self):
    #     os.system("gnome-terminal -- rosrun rosserial_python serial_node.py _baud:=9600 _port:=\"/dev/ttyUSB-IOCtrl\" __name:=\"ioctrl\" ")
    #     self.print_MessStr("Connect to the I/O module controller serial comm.(ttyUSB-IOCtrl)")

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

#===================================================================        

    def onExit(self):
        self.print_MessStr("Exit program...")

        os.system("python3 ~/bin/target_term -run 10 rosnode kill /STM")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 10 rosnode kill /powerctrl")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 10 rosnode kill /stm_starter")
        sleep(0.5)

        sleep(2.0)

        try:            
            self.self.rosserial_proc.kill()
        except:
            print("ros serial process has died...")  

        os.system("python3 ~/bin/target_term -run 10 rosnode kill /camera_flir_node")
        sleep(1.0)   

        os.system("python3 ~/bin/target_term -run 21 rosnode kill /rosout")
        sleep(1.0)

       

        os.system("python3 ~/bin/target_term -run 1 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 2 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 3 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 4 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 5 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 6 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 7 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 8 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 9 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 10 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 11 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 12 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 13 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 14 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 15 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 16 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 17 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 18 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 19 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 20 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 21 exit")
        sleep(0.5)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        self.close()


app = QApplication(sys.argv)
window = MyWindow()
window.show()
sys.exit(app.exec_())

