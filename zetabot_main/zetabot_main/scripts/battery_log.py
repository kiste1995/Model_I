#! /usr/bin/env python

import rospy

import csv
import time
import sys, tty, select, termios, os
from zetabot_main.msg import BatteryInformationMsgs
from std_msgs.msg import Float32
battery1 = BatteryInformationMsgs()
battery2 = BatteryInformationMsgs()

battery_cnt = 2

log_directory = "/home/zetabank/robot_log/battery_log"

battery_SOC_pub = rospy.Publisher("/battery_SOC",Float32,queue_size=10)

today = time.strftime('%Y_%m_%d', time.localtime(time.time()))
file_name = log_directory + "/batt_log_"+today+".csv"

def new_file(today) :
    global file_name

    file_name = log_directory + "/batt_log_"+today+".csv"

    if os.path.isfile(file_name):
        print('ok')
    else :
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        f = open(file_name,'w')
        wr = csv.writer(f)
        log_name = ['Time'] + [j + i for i in BatteryInformationMsgs.__slots__  for j in ['BAT1_','BAT2_']]
        wr.writerow(log_name)
        f.close()


def battery_callback(msg) :
    global battery1
    global battery2
    if msg.id == 0x60 :
        battery1 = msg
    else :
        battery2 = msg

    battery_SOC = (battery1.SOC + battery2.SOC) / battery_cnt

    battery_SOC_pub.publish(battery_SOC)

def main():
    global today
    global file_name

    rospy.init_node("batt_log")

    battery_log_save_flag = rospy.get_param("battery_log_save_flag",True)

    batt_sub = rospy.Subscriber("/battery",BatteryInformationMsgs,battery_callback)


    today = time.strftime('%Y_%m_%d', time.localtime(time.time()))

    yy_mm_dd = time.strftime('%Y-%m-%d', time.localtime(time.time()))


    minit = 99

    rospy.sleep(1)

    new_file(today)

    rospy.sleep(1)


    
    try :
        while (rospy.is_shutdown) :
            if today != time.strftime('%Y_%m_%d', time.localtime(time.time())) :
                today = time.strftime('%Y_%m_%d', time.localtime(time.time()))
                new_file(today)

            if battery_log_save_flag :
                while time.localtime(time.time()).tm_min == minit :
                        rospy.sleep(2)

                f = open(file_name,'a')
                wr = csv.writer(f)

                yy_mm_dd = time.strftime('%Y-%m-%d', time.localtime(time.time()))
                now_time = yy_mm_dd + " " + str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)

                log = [now_time]
                for i in range(len(battery1.__getstate__())) :
                    log.append(battery1.__getstate__()[i])
                    log.append(battery2.__getstate__()[i])


                wr.writerow(log)
                f.close()
                print(log)

                minit = time.localtime(time.time()).tm_min
            else :
                rospy.sleep(1)

        # rospy.spin()

        print("done")



    except:
        print("exit")

        # except Exception:
        #     print("unknown error")
        
    print("Done")


if __name__ == "__main__" :
    main()