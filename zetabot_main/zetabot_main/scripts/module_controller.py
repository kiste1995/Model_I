#!/usr/bin/env python
from time import time
import rospy

from std_msgs.msg import UInt16,UInt64,Bool
from zetabot_main.msg import PowerControlMsgs

from zetabot_main.srv import ModuleControllerSrv
from threading import Thread

#-------------------------------------------------
# from zetabot_main.srv import ModuleControllerSrv

# module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)


# module_controller_srv("uvc_on,led_off,led_green,air_lv2,air_off")
#-------------------------------------------------

purifier_topic = "/purifier_control_command"
uvc_control_topic = "/power_control_command" #port 3
led_topic = "/led_control_command"

uvc_control_pub = rospy.Publisher(uvc_control_topic,PowerControlMsgs,queue_size=10)

uvc_control_msg = PowerControlMsgs()
uvc_control_msg.port = 3

purifier_pub = rospy.Publisher(purifier_topic,UInt16,queue_size=10)
led_command_pub = rospy.Publisher(led_topic,UInt64,queue_size=10)

purifier_command = UInt16()
led_command = UInt64()


pulifier_level = {
    "off" : 0,
    "lv1" : 100,
    "lv2" : 250,
    "lv3" : 400
}

class Led :
    class Color:
        stay = 0x000000
        white = 0xffffff
        blue = 0x0000ff
        sky = 0x00ffff
        green = 0x00ff00
        yellow = 0xffff00
        red = 0xff0000
        orange = 0xff7800

    # class Color:
    #     stay = 0x000000
    #     white = 0xFFFFFF
    #     blue = 0x0000FF
    #     sky = 0x00FFFF
    #     green = 0x00FF00
    #     yellow = 0xFFFF00
    #     red = 0xE51400
    #     orange = 0xF0A30B

    class Mode:
        off = 0x0
        on = 0x1
        blink = 0x2
        blink_fast = 0x3
        fade = 0x4
        sweep = 0x5
        sweep_fast = 0x6
        stay = 0xff

f_color_, b_color_, f_mode_, b_mode_ = Led.Color.white, Led.Color.white, Led.Mode.on, Led.Mode.on

test_f_color_, test_b_color_, test_f_mode_, test_b_mode_ = Led.Color.white, Led.Color.white, Led.Mode.on, Led.Mode.on

command_ = 0

led_test_mode = False

def led_controll(msg) :
    global f_color_, b_color_, f_mode_, b_mode_
    global test_f_color_, test_b_color_, test_f_mode_, test_b_mode_
    global led_test_mode
    global command_
    f_color, b_color, f_mode, b_mode = 0,0,0,0

    comm = msg.split("_")
    print(comm)
    led_mode = comm[0]
    if len(comm) <= 2 :
        f_mode = comm[1]
    else :
        f_mode, b_mode,f_color,b_color = hex(int(comm[1])),hex(int(comm[2])),hex(int(comm[3])),hex(int(comm[4]))
        f_mode, b_mode,f_color,b_color = int(f_mode,16), int(b_mode,16),int(f_color,16),int(b_color,16)

    if led_mode == "ledtest" :
        led_test_mode = True
        if f_mode == "off" :
            led_test_mode = False
            f_color, b_color, f_mode, b_mode = f_color_, b_color_, f_mode_, b_mode_
            test_f_color_, test_b_color_, test_f_mode_, test_b_mode_ = Led.Color.white, Led.Color.white, Led.Mode.on, Led.Mode.on
            return 0
        else :
            if f_color == Led.Color.stay :
                f_color = test_f_color_
            if b_color == Led.Color.stay :
                b_color = test_b_color_
            if f_mode ==Led.Mode.stay :
                f_mode = test_f_mode_
            if b_mode ==Led.Mode.stay :
                b_mode = test_b_mode_

            test_f_color_, test_b_color_, test_f_mode_, test_b_mode_ = f_color, b_color, f_mode, b_mode

        command = ((f_mode << 24) | f_color) << 32 | ((b_mode << 24) | b_color)

    else:
        if f_color == Led.Color.stay :
            f_color = f_color_
        if b_color == Led.Color.stay :
            b_color = b_color_
        if f_mode ==Led.Mode.stay :
            f_mode = f_mode_
        if b_mode ==Led.Mode.stay :
            b_mode = b_mode_
        
        f_color_, b_color_, f_mode_, b_mode_ = f_color, b_color, f_mode, b_mode
        
        if not(led_test_mode) :
            command = ((f_mode << 24) | f_color) << 32 | ((b_mode << 24) | b_color)
        else :
            return 0
    
    if command == command_ :
        return 0
    
    pre_time = time()
    while time() - pre_time <= 3 :
        led_command_pub.publish(command)
        try :
            sub_led_control_command_ack = rospy.wait_for_message("/led_control_command_ack",UInt64,timeout=0.5)
        except :
            sub_led_control_command_ack = UInt64()
        if command == sub_led_control_command_ack.data :
            break

    command_ = command
    print(hex(command))

def module_controller(comm_list):
    global module_controll_command
    global blink_flag
    global blink_term
    global led_color
    global led_count

    comm_list = comm_list.command.split(",")

    for i in comm_list :
        comm = i.split("_")
        print(i)
        if "led" in i :
            led_controll(i)

        elif "air" in i :
            print("air")
            purifier_command.data = pulifier_level[comm[1]]
            purifier_pub.publish(purifier_command)
        
        elif "uvc" == comm[0] :
            if "_on" in i :
                uvc_control_msg.state = True
            elif "_off" in i :
                uvc_control_msg.state = False
            uvc_control_pub.publish(uvc_control_msg)    
            print("")

    return (str(comm_list))    


def main() :
    rospy.init_node("module_controller_server")

    rospy.sleep(1)

    srv = rospy.Service('module_controller_srv', ModuleControllerSrv, module_controller)
    print("Module_control Ready")
    print("-"*20)

    rospy.spin()

if __name__ == "__main__":
    main()
