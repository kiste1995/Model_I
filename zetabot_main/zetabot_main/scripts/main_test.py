#!/usr/bin/env python

import rospy
import sys
import socket
from threading import Thread
import json
from zetabot_main.srv import Dbsrv
from zetabot_main.srv import MoveBaseSrv
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import zetabot_main.msg
import actionlib
import testhash


### configurations that can be changed
# gHost = "192.168.0.48"  # "localhost"			# "192.168.0.123"
# gHost = "192.168.0.28"
gHost = "192.168.112.2"
gPort = 7777


# gHost = "192.168.0.50"
gSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

g_send_hashmap = testhash.hashmap()
g_socket_connected = False

class MobeBaseAction(object):
    # create messages that are used to publish feedback/result
    _feedback = zetabot_main.msg.MoveBaseActFeedback()
    _result = zetabot_main.msg.MoveBaseActResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, zetabot_main.msg.MoveBaseActAction, execute_cb=self.execute_cb, auto_start = False)
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
        self._as.start()
      
    def execute_cb(self, req):
        # helper variables
        rate = rospy.Rate(1)
        success = True
        
        # append the seeds for the MoveBaseAct sequence
        self._feedback.feedback = []
        self._feedback.feedback.append(0)
        self._feedback.feedback.append(1)
        
        # publish info to the console for the user
        print(self._feedback.feedback)

        
        self.clear_costmaps_srv()
        
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = req.x
        goal.target_pose.pose.position.y = req.y
        goal.target_pose.pose.orientation.w = req.z

        client.send_goal(goal)
        wait = client.wait_for_result()

        self._result = client.get_result()
        self._as.set_succeeded(self._result)
        
        if not wait:
            rospy.logerr("Action server not available!")
            # rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result() 
        
        # start executing the action
        # for i in range(1, goal.order):
        #     # check that preempt has not been requested by the client
        #     # if self._as.is_preempt_requested():
        #     #     rospy.loginfo('%s: Preempted' % self._action_name)
        #     #     self._as.set_preempted()
        #     #     success = False
        #     #     break
        #     self._feedback.feedback.append(self._feedback.feedback[i] + self._feedback.feedback[i-1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
          
        # if success:
        #     self._result.result = self._feedback.feedback
        #     rospy.loginfo('%s: Succeeded' % self._action_name)
        #     self._as.set_succeeded(self._result)

def re_connet_server():
    global g_socket_connected
    global gSock
    while not g_socket_connected:
        print("reconnecte ")
        gSock.close()
        gSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # attempt to reconnect, otherwise sleep for 2 seconds
        try:
            gSock.connect((gHost, gPort))
            g_socket_connected = True
            print("re-connection successful")
            gThrd = Thread(target=recv_msg, args=(gSock,))  # thread to process received packets
            gThrd.daemon = True
            gThrd.start()
        except socket.error:
            rospy.sleep(2)

def serviceproccess(req):
    print (req)
    global g_send_hashmap
    code = testhash.codecreat()
    g_send_hashmap.add(code,("false",""))

    if g_socket_connected == True:
        # send_packet(req.command,req.data,code)
        send_packet(req.command,req.data,code)
        print("send_packet_done")
        result =''
        if not("_insert" in req.command) and not("create" in req.command) and not("map_data" == req.command) :
            while True:
                # print("1111111111111",req.command)
                if g_send_hashmap.get(code)[0] == "true":
                    result = g_send_hashmap.get(code)[1]
                    g_send_hashmap.delete(code)
                    break
                rospy.sleep(0.2)
    else :
        result = "connect fail"
    print("result :",result)
    return result

def recv_msg(sock):
    global g_send_hashmap
    global g_socket_connected
    MsgRecv = ''
    buffer_size = 1024
    code = ''
    while True :
        try:
            MsgRecv = gSock.recv(1024)
            print("recv_done :", MsgRecv)
            buffer_size = 1024
            if MsgRecv.startswith("zetas") :
                print("zetas")
                buffer_size = int(MsgRecv.split("|")[1])
                code =  MsgRecv.split("|")[5]
                # print "size",buffer_size
                if buffer_size > 1024 :
                    while  buffer_size - 1024 >= 0 :
                        MsgRecv = MsgRecv +  gSock.recv(1024)
                        buffer_size = buffer_size - 1024
                        print("2222222222222222222")
                    print ("\t\t<--- [Server111] {}".format(MsgRecv))
                if not MsgRecv:
                    break
                g_send_hashmap.add(code,("true",MsgRecv.split("|")[6]))
                print("code_true")
            if not MsgRecv:
                break
            print ("\t\t<--- [Server222] {}".format(MsgRecv))

        except Exception as e:
            print('exception!!', e)
            g_socket_connected = False

def connect_socket(host, port):
    global g_socket_connected
    gSock.connect((host, port))  # connect()
    print ("- Connected to " + host + ", " + str(port))
    g_socket_connected = True

    gThrd = Thread(target=recv_msg, args=(gSock,))  # thread to process received packets
    gThrd.daemon = True
    gThrd.start()

def send_packet(command,data,code):
    global gSock
    send_msg = ""
    robot_serial = "test_serial"
    map_name = "test_map"

    data = "zetas" + "|" + robot_serial + "|" + map_name + "|" + command + "|" + code +"|" +data +"|" +"zetae"
    msg_len = len(data)
    msg_list = data.split("|")
    msg_len = msg_len + len(str(msg_len)) + 2

    idx = 0
    while idx != len(msg_list):
        if idx == 0 :
            send_msg = msg_list[idx]
        elif idx == 1:
            send_msg = send_msg + "|" + str(msg_len)
            send_msg = send_msg + "|" + msg_list[idx]
        # elif idx == 2:
        #     send_msg = send_msg + "|" + msg_list[idx]
        # elif idx == 3:
        #     send_msg = send_msg + "|" + msg_list[idx] 
        else:
            send_msg = send_msg + "|" + msg_list[idx]
        idx = idx + 1

    print ('\t---> Sending Packet : ', send_msg)
    gSock.sendall(send_msg)


if __name__ == "__main__":
    rospy.init_node('zetabot_main')
    db_srv = rospy.Service('db_works', Dbsrv, serviceproccess)
    movebase_srv = MobeBaseAction("move_base_action")
    
    if (len(sys.argv) != 3):
        print ("- Usage : python {} <target_host_ip> <port>".format(sys.argv[0]))
        print ("* Exiting...")
    else:
        gHost = sys.argv[1]
        gPort = int(sys.argv[2])  # to integer
        print ("- Protocol Client : \n")

        connect_socket(gHost, gPort)  # connect to server
        #gSock.close()
        print ("- Program exiting normally")

    reconnetThrd = Thread(target=re_connet_server, args=())  # process reconnect to server
    reconnetThrd.daemon = True
    reconnetThrd.start()
    #send_packet("alldata","",testhash.codecreat())
    rospy.spin()