#! /usr/bin/env python

import rospy

import mysql.connector

import json
import csv
import time
import sys, os
from zetabot_main.msg import EnvironmentMsgs
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose


robot_id = ""

battery_SOC = None

mydb = mysql.connector.connect(
    host="db-instance-zetabank.cyqiscqvsgd5.ap-northeast-2.rds.amazonaws.com",
    user="admin",
    passwd="zetabank!2",
    database="zetabank_db" 
)


mc = mydb.cursor()

sql = "INSERT INTO robot1_sensor (robot_id, x, y, ultrafine_dust, fine_dust, co2, formaldehyde, co, no2, radon,tvoc,temperature,humidity,battery) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)"


air_log = EnvironmentMsgs()
pose_log = Pose()
file_ready_flag = False
pose_ready_flag = False

battery_cnt = 2

today = time.strftime('%Y_%m_%d', time.localtime(time.time()))

yy_mm_dd = time.strftime('%Y-%m-%d', time.localtime(time.time()))

log_directory = "/home/zetabank/robot_log/air_log"
file_name = log_directory + "/air_log_"+today+".csv"

def new_file(today) :
    global file_ready_flag
    global file_name

    file_ready_flag = False
    file_name = log_directory + "/air_log_"+today+".csv"

    if os.path.isfile(file_name):
        print('ok')
    else :
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        f = open(file_name,'w')
        wr = csv.writer(f)
        log_name = ['Time'] + ['x'] + ['y'] + [i for i in EnvironmentMsgs.__slots__]
        wr.writerow(log_name)
        f.close()
    
    file_ready_flag = True

def air_callback(msg) :
    global mc
    global sql
    global mydb
    global robot_id

    global air_log
    global today

    global battery_SOC
    air_log = msg

    Fine_dust = msg.Dust_PM10_ugm3
    Ultrafine_dust =  msg.Dust_PM2_5_ugm3
    CO2 = msg.CO2_ppm
    Formaldehyde =  msg.HCHO_ugm3
    CO = msg.CO_ppm
    NO2 = msg.NO2_ppm
    Radon = msg.Rn_Bqm3
    Organic_compounds = msg.TVOCs_ugm3
    Temperature = msg.temp_celcius
    Humidity = msg.hum_RHp
    battery = battery_SOC

    val = (robot_id,pose_log.position.x,pose_log.position.y,Ultrafine_dust,Fine_dust,CO2,Formaldehyde,CO,NO2,Radon,Organic_compounds,Temperature,Humidity,battery)
    
    try : 
        mc.execute(sql, val)

        mydb.commit()
    except:
        print("db error")

    print(mc.rowcount, "insert record")


    if today != time.strftime('%Y_%m_%d', time.localtime(time.time())) :
        today = time.strftime('%Y_%m_%d', time.localtime(time.time()))
        new_file(today)

    if file_ready_flag and pose_ready_flag :
        f = open(file_name,'a')
        wr = csv.writer(f)

        yy_mm_dd = time.strftime('%Y-%m-%d', time.localtime(time.time()))
        now_time = yy_mm_dd + " " + str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min) + ":" + str(time.localtime(time.time()).tm_sec)
        log = [now_time]
        log.append(pose_log.position.x)
        log.append(pose_log.position.y)
        for i in range(len(air_log.__getstate__())) :
            log.append(air_log.__getstate__()[i])


        wr.writerow(log)
        f.close()
        print(log)

def pose_callback(msg) :
    global pose_log
    global pose_ready_flag
    pose_log = msg

    pose_ready_flag = True

def battery_callback(msg) :
    global battery_SOC

    battery_SOC = msg.data

def main():
    global robot_id

    

    rospy.init_node("air_log")

    batt_sub = rospy.Subscriber("/battery_SOC",Float32,battery_callback)

    air_log_save_flag = rospy.get_param("air_log_save_flag",True)
    robot_id = rospy.get_param("robot_id","")

    air_sub = rospy.Subscriber("/air",EnvironmentMsgs,air_callback)
    pose_sub = rospy.Subscriber("/robot_pose",Pose,pose_callback)


    rospy.sleep(1)

    new_file(today)

    rospy.spin()


if __name__ == "__main__" :
    main()