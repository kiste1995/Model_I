#!/usr/bin/env python

from logging import log
from sqlite3.dbapi2 import connect
import rospy
import roslib
import actionlib

import sys
import os
import math
import numpy
import threading
import time, csv

import full_coverage.msg
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from zetabot_main.srv import InitPoseSrv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from full_coverage.msg import lidar_filter 
from zetabot_main.srv import ModuleControllerSrv
from zetabot_main.msg import SonarArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


sonar_warning_top = 40
sonar_warning_bottom = 35

sonar_stop_top = 30
sonar_stop_bottom = 25

wall_distance = 0.25
max_a_val = 0.2
x_val = 0.1

pose = Pose()
robot_z = 0
twist = Twist()
key = 1
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

bumper_flag = False
estop_flag = False
MVStop_flag = False
imu_avr_term = None
imu_get_list = []
imu_get_time = None
stm_called = True
stm_recall_flag = False
set_pose_flag = True
disconnect_flag = False
odom_pose = {}

connect_ = 'STM_connected'
disconnect = 'STM_disconnected'

emergency_topic = "/emergency_stop"
emergency_pub = rospy.Publisher(emergency_topic,String, queue_size=10)
emergency_call = rospy.ServiceProxy("/emergency_stm", Empty)
odom_pose_srv = rospy.ServiceProxy('/init_pose_srv', InitPoseSrv)
module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)
log_directory = "/home/zetabank/robot_log/stm_log"

rospy.init_node('sonar_filter')

class sonar_sensor :
    sonar_UF = 0
    sonar_UR = 0
    sonar_UB = 0
    sonar_UL = 0
    sonar_DF = 0
    sonar_DR1 = 0
    sonar_DR2 = 0
    sonar_DB1 = 0
    sonar_DB2 = 0
    sonar_DL1 = 0
    sonar_DL2 = 0

def recv_sonar(data):
    sonar_sensor.sonar_UF = data.data[5]
    sonar_sensor.sonar_UR = data.data[6]
    sonar_sensor.sonar_UB = data.data[7]
    sonar_sensor.sonar_UL = data.data[10]
    sonar_sensor.sonar_DF = data.data[4]
    sonar_sensor.sonar_DR1 = data.data[1]
    sonar_sensor.sonar_DR2 = data.data[0]
    sonar_sensor.sonar_DB1 = data.data[9]
    sonar_sensor.sonar_DB2 = data.data[8]
    sonar_sensor.sonar_DL1 = data.data[2]
    sonar_sensor.sonar_DL2 = data.data[3]

def recv_bumper(data):
    global bumper_flag
    bumper_flag = data.data

def recv_estop(data):
    global estop_flag
    estop_flag = data.data

def recv_MBStop(data):
    global MVStop_flag
    MVStop_flag = data.data

def recv_imu(data):
    global imu_avr_term
    global imu_get_list
    global imu_get_time
    global stm_called
    global stm_recall_flag
    stm_called = True
    stm_recall_flag = False


    if imu_avr_term == None :
        imu_get_list.append(rospy.get_time())
        if len(imu_get_list) >= 5 :
            imu_avr_term = (imu_get_list[-1] - imu_get_list[0]) / 4
            imu_get_time = imu_get_list[-1]

    else :
        imu_get_time = rospy.get_time()

def emergency_stm_call():
    emergency_call()
     
def odom_callback(msg):
    global odom_pose
    odom_pose = {}
    odom_pose = {
        'position_x' : msg.position.x,
        'position_y' : msg.position.y,
        'orientation_z' : msg.orientation.z,
        'orientation_w' : msg.orientation.w
    }

def odom_position_srv(odom_pose_,file_name):
    global disconnect_flag
    global log_directory
    odom_pose_srv(odom_pose_['position_x'],odom_pose_['position_y'],odom_pose_['orientation_z'],odom_pose_['orientation_w'])

    if not os.path.exists(log_directory):
        os.makedirs(log_directory)
    f = open(file_name, 'a')
    wr = csv.writer(f)
    log_name = [str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)]
    log_name.append(disconnect)
    wr.writerow(log_name)
    f.close
    disconnect_flag = True

def new_file(file_name):
    if os.path.isfile(file_name):
        pass
    else:
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        f = open(file_name, 'w')
        wr = csv.writer(f)
        log_name = ['time']
        log_name.append('status')
        wr.writerow(log_name)
        f.close

def emergency_send () :
    global bumper_flag
    global estop_flag
    global stm_called
    global stm_recall_flag
    global set_pose_flag
    global odom_pose
    global log_directory
    global disconnect_flag

    today = time.strftime('%Y_%m_%d', time.localtime(time.time()))
    yy_mm_dd = time.strftime('%Y-%m-%d', time.localtime(time.time()))

    while True:
        rospy.sleep(0.01)
        emergency_msg = ""
        # os.system("clear")
        print("estop_flag : ",estop_flag)

        if bumper_flag == True : 
            emergency_msg += "/bumper_stop"

        if estop_flag == True :
            emergency_msg += "/emergency_button_stop"

        if MVStop_flag == True :
            emergency_msg += "/move_base_stop"

        if imu_avr_term != None :
            print(rospy.get_time() - imu_get_time)
            print(imu_avr_term)
            print(imu_avr_term + (imu_avr_term/2))
            print(imu_get_list)
        if imu_avr_term == None :
            emergency_msg += "/imu_error_stop"

        elif rospy.get_time() - imu_get_time >= (imu_avr_term*3) :
            emergency_msg += "/imu_error_stop"

            stm_called = False

        if not stm_called and not stm_recall_flag:
            stm_recall_flag = True
            print("recall")
            set_pose_flag = False

            t1 = threading.Thread(target=emergency_stm_call)
            t1.daemon = True 
            t1.start()
            print("return")

        if not set_pose_flag and stm_called:
            file_name = log_directory + "/stm_log_" + today + ".csv"
        #     print("main: ",odom_pose)
            if today != time.strftime('%Y_%m_%d', time.localtime(time.time())) :
                today = time.strftime('%Y_%m_%d', time.localtime(time.time()))
                
                new_file(file_name)
                
            odom_position_srv(odom_pose, file_name)

            if disconnect_flag:
                if not os.path.exists(log_directory):
                    os.makedirs(log_directory)
                f = open(file_name, 'a')
                wr = csv.writer(f)
                log_name = [str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)]
                log_name.append(connect_)
                wr.writerow(log_name)
                f.close
                set_pose_flag = True

        emergency_pub.publish(emergency_msg)

def main():

    t1 = threading.Thread(target=emergency_send)
    t1.daemon = True 
    t1.start()

    bumper_topic = "/bumper"
    rospy.Subscriber(bumper_topic,Bool,recv_bumper)

    estop_topic = "/estop"
    rospy.Subscriber(estop_topic,Bool,recv_estop)

    move_base_stop_topic = "/move_base_stop"
    rospy.Subscriber(move_base_stop_topic,Bool,recv_MBStop)

    imu_topic = "/imu"
    rospy.Subscriber(imu_topic,Imu,recv_imu)

    pose_topic = "/robot_pose"
    rospy.Subscriber(pose_topic, Pose, odom_callback)
    
    rospy.spin()

if __name__ == "__main__":
    main()