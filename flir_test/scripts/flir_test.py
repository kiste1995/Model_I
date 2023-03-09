#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np, cv2
import rospy, os, threading, time, sys, math
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32MultiArray
from cv_bridge import CvBridge,CvBridgeError

# uint32 height
# uint32 width

bridge = CvBridge()

temp_center = 0
data = Image()
temp = 0
class flir_show():
    
    def __init__(self):
        self.flir_rgb_sub = rospy.Subscriber('/camera_flir_node/rgb/image_raw', Image, self.callback_rgb)
        self.flir_8b_sub = rospy.Subscriber('/camera_flir_node/ir_8b/image_raw', Image, self.callback_ir)
        self.flir_16b_sub = rospy.Subscriber('/camera_flir_node/ir_16b/image_raw', Image, self.callback_ir_color)
        self.flir_pix = rospy.Subscriber('/camera_flir_node/thermal/temperature', UInt32MultiArray, self.callback_pix)

    def callback_pix(self, msg):
        global temp_center

        PlanckR1 =  16528.178
        PlanckB  = 1427.5
        PlanckF  = 1.0
        PlanckO  = -1307.0
        PlanckR2 =  0.012258549
        Center = int(len(msg.data)/2)

        TempReflected = 20.0
        Emissivity = 0.95

        pix_tmp = msg.data[Center] * 4

        RAWrefl=PlanckR1/(PlanckR2*(math.exp(PlanckB/(TempReflected+273.15))-PlanckF))-PlanckO

        RAWobj=(pix_tmp-(1-Emissivity)*RAWrefl)/Emissivity

        result = PlanckB/math.log(PlanckR1/(PlanckR2*(RAWobj+PlanckO))+PlanckF)-273.15

        temp_center = result
        # print("result : ",result)

    # #    print(len(msg.data))
    #     center
    #     center = int(len(msg.data)/2)/10.0 -273.15
    #     print("pix :", a/30.0)

    def callback_rgb(self, msg):
        red_color = (0, 255, 0)
        cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
        img_array = np.array(cv_image, dtype = np.dtype('f8'))
        cir = cv2.circle(cv_image, (msg.width/2, msg.height/2), 50, red_color)
        flip = cv2.rotate(cir,cv2.ROTATE_90_CLOCKWISE)
        res = cv2.resize(flip, dsize=(540, 540), interpolation=cv2.INTER_AREA)
        cv2.imshow('rgb', res)
        cv2.waitKey(1)

    def callback_ir(self, msg):

        cv_image = bridge.imgmsg_to_cv2(msg,msg.encoding)
        # cv2.imshow('ir_8b', cv_image)
        # cv2.waitKey(1)
        
    def callback_ir_color(self, msg):
        global data
        data = msg
        cv_image = bridge.imgmsg_to_cv2(msg,msg.encoding)

        dx = msg.width/2
        dy = msg.height/2

        temp_center = (cv_image[dy,dx]/10.0) - 273.15
        temp = str(temp_center/3.2)

        img_np = np.array(cv_image, dtype = np.dtype('uint8'))
        flip = cv2.rotate(img_np,cv2.ROTATE_90_CLOCKWISE)
        cv2.putText(flip, temp, (10,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1,cv2.LINE_AA)
        res = cv2.resize(flip, dsize=(540, 540), interpolation=cv2.INTER_AREA)
        cv2.imshow('ir_16b', res)
        cv2.waitKey(1)
        # rospy.sleep(3)


def thread_():
    t1 = threading.Thread(target = thermal_voice)      
    t1.setDaemon(True)
    t1.start()

def thermal_voice():
    global data

    while True:
        try:
            msg = data
            dx = msg.width/2
            dy = msg.height/2

            cv_image = bridge.imgmsg_to_cv2(msg,msg.encoding)

            temp_center = (cv_image[dy,dx]/10.0) - 273.15
            temp = temp_center/3.2 
            print("temp : ",temp)
            # print("temp_pix",temp_center)
            rospy.sleep(0.1)
            os.system("clear")
            
            if temp > 36.5:
                os.system("mplayer" + " ~/thermal_voice.mp3 &")
                os.system("cat")
                print(temp)
                sleep(8)
        except:
            print('e')


if __name__=='__main__':
    rospy.init_node("flir_show")
    flir_show()
    thread_()
    rospy.spin()