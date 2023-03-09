#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import os
import time
from zetabot_main.srv import Dbsrv
from full_coverage.srv import Cell2pose
from std_msgs.msg import String
from zetabot_main.msg import EnvironmentMsgs
from zetabot_main.srv import ModuleControllerSrv

module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)

cell_name = ""
today = time.strftime('%Y_%m_%d', time.localtime(time.time()))


def cell_name_recv(msg) :
    global cell_name
    cell_name = msg.data

def make_air_class_file(air_info_dict,air_class) :
    global today
    
    air_class_dict_list = []
    air_class_file_path = "/home/zetabank/catkin_ws/src/full_coverage/scripts/map/air_class_"+today+".json"

    # air_info_average = (int(air_info_dict["Fine_dust"]) + int(air_info_dict["Ultrafine_dust"]) + int(air_info_dict["Organic_compounds"]))/3
    # air_class = "a" if air_info_average <= 1 else("b" if 1 < air_info_average <= 3 else "c")

    air_class_dict = {
        "cell_name" : cell_name,
        "class" : air_class
    }

    try :
        with open(air_class_file_path, 'r') as air_class_file:
            air_class_dict_list = json.load(air_class_file)
    except :
        pass

    for i in air_class_dict_list :
        if i['cell_name'] == air_class_dict['cell_name'] :
            i['class'] = air_class_dict['class']
            air_class_dict = dict()
            break
    
    if air_class_dict :
        air_class_dict_list.append(air_class_dict)
        
    with open(air_class_file_path, 'w') as air_class_file:
        json.dump(air_class_dict_list, air_class_file, indent=4)
        print("write air class")

def make_air_info_file(air_info_dict) :
    global today

    air_info_file_path = "/home/zetabank/catkin_ws/src/full_coverage/scripts/map/air_info_"+today+".json"
    air_info_dict_list = []

    try :
        with open(air_info_file_path, 'r') as air_info_file:
            air_info_dict_list = json.load(air_info_file)
    except :
        pass

    air_info_dict_list.append(air_info_dict)
        
    with open(air_info_file_path, 'w') as air_info_file:
        json.dump(air_info_dict_list, air_info_file, indent=4)
        print("write air info")

def air_info_recv(msg) :
    global cell_name

    air_info_dict = dict()
    

    air_info_dict = {
            "cell_name" : cell_name,
            "Fine_dust" : msg.Dust_PM10_ugm3,
            "Ultrafine_dust" : msg.Dust_PM2_5_ugm3,
            # "time" : str(time.localtime(time.time()).tm_hour) +"h, " + str(time.localtime(time.time()).tm_min) +"m, " + str(time.localtime(time.time()).tm_sec)+"s",
            "CO2" : msg.CO2_ppm,
            "Formaldehyde" : msg.HCHO_ugm3,
            "CO" : msg.CO_ppb,
            "NO2" : msg.NO2_ppb,
            "Radon" : msg.Rn_mBqm3,
            "Organic_compounds" : msg.TVOCs_ugm3,
            "Temperature" : msg.temp_celcius,
            "Humidity" : msg.hum_RHp
    }


    print(air_info_dict)

    if int(air_info_dict["Fine_dust"]) > 100 or int(air_info_dict["Ultrafine_dust"]) > 100 :
        air_class = "c"
        module_controller_srv("air_lv3_on")
        print(air_info_dict["Fine_dust"],air_info_dict["Ultrafine_dust"])
    elif int(air_info_dict["Fine_dust"]) < 50 or int(air_info_dict["Ultrafine_dust"]) < 50 :
        air_class = "a"
        module_controller_srv("air_lv1_on")
    else :
        air_class = "b"
        module_controller_srv("air_lv2_on")

    # if not air_info_dict["sensor"][0]["cell_name"] == "empty" :
    if air_info_dict["cell_name"] == "B3" or air_info_dict["cell_name"] == "B7" or air_info_dict["cell_name"] == "B13" :
        make_air_class_file(air_info_dict,air_class)
        make_air_info_file(air_info_dict)
        # dbsrv = rospy.ServiceProxy('db_works', Dbsrv)
        # resp1 = dbsrv("air_info_insert",str(air_info_dict))
        print("send_server")


def main():
    rospy.init_node("test")

    # cli = rospy.ServiceProxy("cell2pose",Cell2pose)
    # cell_condition = cli("A1")

    cell_name_topic = "/cell_name"
    rospy.Subscriber(cell_name_topic,String,cell_name_recv)

    # air = "0|1|2|3|4|5|6|7|8|9"

    air_info_topic = "/air"
    rospy.Subscriber(air_info_topic,EnvironmentMsgs,air_info_recv)


    # print(cell_condition)

    rospy.spin()

if __name__ == "__main__":
    main()
