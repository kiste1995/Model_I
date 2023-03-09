#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os,sys,threading, time
from time import sleep
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

class Ros_check:
    def __init__(self, imu_topic):
        self.check_flag = False
        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.callback)
    def callback(self, msg):
        self.check_flag = True

class Command:
    
    def __init__(self):    
        self.command_rosrun = "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM{} _baud:=460800 __name:='STM'"
        self.command_permission = "echo 'zetabank' | sudo -S chmod 777 /dev/ttyACM{}"
        # self.echo = "echo 'zetabank' | sudo -S tee"
        self.command_root = "whoami"
        self.number = 99
        
        self.imu_error_service = rospy.Service('/emergency_stm', Empty, self.ACM_check)

    def ACM_check(self,msg = EmptyRequest()):
        sleep(0.2)
        os.system('rosnode kill /STM')
        while True:          
            lsusb = self.STM_usb()
            if lsusb:
                print('STM_on')
                if self.number != 99: 
                    sleep(2)
                    self.ACM_double_check(self.number)
                    return EmptyResponse()

                else :
                    for i in range(2):        
                        t1 = self.ACM_double_check(i)
                    #ros
                        imu_topic = '/imu'
                        ros_check = Ros_check(imu_topic)
                        sleep(10)

                        if not ros_check.check_flag:
                            nodes = os.popen("rosnode list").readline()
                            os.system("rosnode kill /STM")
                            t1.join()
                        else:
                            self.number = i       
                            break  
                break  
            else:
                pass
                # return EmptyResponse()

    def ACM_double_check(self,number): 

        root = os.popen("whoami").read()
        
        if 'zetabank' in root:
            # command = "echo 'zetabank' | sudo -S tee"
            # os.system(command)
            os.system(self.command_permission.format(number))

        else :
            os.system(self.command_permission.format(number))

        command_rosrun = self.command_rosrun.format(number)
        t1 = threading.Thread(target= self.ACM_thread, name="ACM thread", args=(command_rosrun,))
        t1.setDaemon(True)
        t1.start()
        return t1   

    def STM_usb(self):
        lsusb_list = os.popen("lsusb").read()
        if 'STM' in lsusb_list:
            return True
        else:
            return False

    def ACM_thread(self,command_rosrun):
        os.system(command_rosrun)

if __name__=='__main__':
    rospy.init_node("stm_starter")
    acm_command = Command()

    acm_command.ACM_check()

    rospy.spin()