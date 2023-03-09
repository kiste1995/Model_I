#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import threading

import sys, select, termios, tty

import time

import os

msg = """
Control Your Zetabot!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
"""
incdec_val = 0.03

speak_pub = rospy.Publisher("/speak",Bool,queue_size=5)

roming_flag = False
repeat_flag  = False

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def roming_thd_call_back():
    global roming_flag
    roming_vel = 0.3
    during = 5
    rate = 0.02

    while True :
        while roming_flag :
            now = time.time()
            control_linear_vel  = -1 * roming_vel
            roming_vel = control_linear_vel
            control_angular_vel = 0
            twist = Twist()
            twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_vel
            while time.time() - now <= during :
                pub.publish(twist)
                rospy.sleep(rate)
        rospy.sleep(0.1)

def repeat_thd_call_back():
    global repeat_flag
    global speak_pub
    num = 0

    sound_list = ['sba_good_day','sba_mask','sba_welcome']
 
    while True :
        while repeat_flag :
            now = time.time()
            if num == len(sound_list) :
                num = 0 
            
            while time.time() - now <= 30 and repeat_flag:
                print(time.time()-now)
                rospy.sleep(0.1)
            if repeat_flag:
                speak_pub.publish(True)
                command = "mplayer /home/zetabank/catkin_ws/src/sound/" + sound_list[num] + ".mp3"
                os.system(command)
                speak_pub.publish(False)
                num += 1
	rospy.sleep(0.1)


if __name__=="__main__":
    global roming_flag
    global repeat_flag

    settings = termios.tcgetattr(sys.stdin)
    


    rospy.init_node('zetabank_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    teleop_pub = rospy.Publisher('/teleop', Bool, queue_size=5)

    status = 0
    target_linear_vel = 0
    target_angular_vel = 0
    control_linear_vel = 0
    control_angular_vel = 0

    roming_thd = threading.Thread(target=roming_thd_call_back)
    roming_thd.daemon = True 
    roming_thd.start()

    repeat_thd = threading.Thread(target=repeat_thd_call_back)
    repeat_thd.daemon = True
    repeat_thd.start()

    try :
        print msg
        while(1):
            key = getKey()
            if key == '1' :
                os.system("mplayer /home/zetabank/catkin_ws/src/say/1_deputy_mayor.mp3")
            elif key == '2':
                os.system("mplayer /home/zetabank/catkin_ws/src/say/5_deputy_mayor.mp3")
	    elif key == '0':
		repeat_flag = not(repeat_flag) 
		if repeat_flag : 
		    print("repeat_on")
		else :
		    print("repeat_off")
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        teleop_bool = Bool()
        teleop_bool.data = 1

        pub.publish(twist)
        teleop_pub.publish(teleop_bool)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
