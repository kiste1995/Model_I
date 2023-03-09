#!/usr/bin/env python
import rospy
import math
import time
import os
import csv

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu


import sqlite3



file_ready_flag = False


today = time.strftime('%Y_%m_%d', time.localtime(time.time()))

yy_mm_dd = time.strftime('%Y-%m-%d', time.localtime(time.time()))

log_directory = "/home/zetabank/robot_log/imu_test_log"
imu_file_name = log_directory + "/imu_test_log_"+today+".csv"
pose_file_name = log_directory + "/pose_test_log_"+today+".csv"

def quaternion_to_euler_angle(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

def g_rangle_range_tr(euler_z):

    angle = 0
    if euler_z > 0:
        angle = euler_z
    else:
        angle = 180 + euler_z + 179

    return angle

def angle_scailing(z) :
    z = z-90
    
    if z<0 :
        return z + 360
    else :
        return z


def pose_send(val) :

    pose = val
    X, Y, Z = quaternion_to_euler_angle(pose.orientation)
    robot_z_comp = g_rangle_range_tr(Z)
    robot_z = angle_scailing(robot_z_comp)

    if file_ready_flag :
        # Insert a row of data
        con = sqlite3.connect('example.db')
        cur = con.cursor()

        now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min) + ":" + str(time.localtime(time.time()).tm_sec)

        command = "INSERT INTO robot_pose VALUES ('"+ now_time + "' , '" + str(robot_z) +"')"
        cur.execute(command)

        # Save (commit) the changes
        con.commit()

        # We can also close the connection if we are done with it.
        # Just be sure any changes have been committed or they will be lost.
        con.close()

def imu_CB(val) :
    imu = val
    X, Y, Z = quaternion_to_euler_angle(imu.orientation)
    robot_z_comp = g_rangle_range_tr(Z)
    robot_z = angle_scailing(robot_z_comp)

    if file_ready_flag :
        con = sqlite3.connect('example.db')
        cur = con.cursor()

        now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min) + ":" + str(time.localtime(time.time()).tm_sec)

        command = "INSERT INTO imu VALUES ('"+ now_time + "' , '" + str(robot_z) +"')"
        cur.execute(command)

        # Save (commit) the changes
        con.commit()

        # We can also close the connection if we are done with it.
        # Just be sure any changes have been committed or they will be lost.
        con.close()

def main():
    global file_ready_flag


    rospy.init_node("imu_test_log")


    robot_pose_topic = "/robot_pose"
    rospy.Subscriber(robot_pose_topic,Pose,pose_send)

    imu_topic = "/imu"
    rospy.Subscriber(imu_topic,Imu,imu_CB)
    

    if os.path.isfile(imu_file_name):
        print('ok')
    else :
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        f = open(imu_file_name,'w')
        wr = csv.writer(f)
        log_name = ['Time'] + ['imu_angle']
        wr.writerow(log_name)
        f.close()

    if os.path.isfile(pose_file_name):
        print('ok')
    else :
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        f = open(pose_file_name,'w')
        wr = csv.writer(f)
        log_name = ['Time'] + ['pose_angle']
        wr.writerow(log_name)
        f.close()
    
    file_ready_flag = True

    rospy.sleep(1)


    print("turn_ready")

    rospy.spin()
    

if __name__ == "__main__":
    main()