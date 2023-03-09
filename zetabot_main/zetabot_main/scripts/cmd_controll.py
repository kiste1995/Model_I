#!/usr/bin/env python
import rospy
import actionlib

from threading import Thread
import os
import time

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from zetabot_main.msg import MoveMsgs, MoveBaseActAction, MoveBaseActFeedback, MoveBaseActResult, MoveBaseActGoal
from actionlib_msgs.msg import GoalStatusArray

from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RosMaker():
    def __init__(self,type,topic_name,message_type):
        self._topic_name = topic_name
        self._message_type = message_type
        if type == "pub" :
            self._pub = rospy.Publisher(self._topic_name,self._message_type,queue_size=10)

        elif type == "sub" :
            self.msg = message_type()
            self._sub = rospy.Subscriber(self._topic_name,self._message_type,self.callback)

        elif type == "action" :
            self._action_name = topic_name

            message_type_str = str(message_type).split(".")[-1].split("'")[0]
            self.msg = globals()[message_type_str.replace("Action","Goal")]()
            self._feedback = globals()[message_type_str.replace("Action","Feedback")]()
            self._result = globals()[message_type_str.replace("Action","Result")]()

            self._as = actionlib.SimpleActionServer(self._action_name, message_type, execute_cb=self.execute_cb, auto_start = False)
            self._as.start()
        
        self.goal = MoveBaseActGoal()
        self.action_flag = False
        # battery_level_topic = "/battery_level"
        # battery_level_pub = rospy.Publisher(battery_level_topic,UInt8,queue_size=10)
 
    # instance method
    def callback(self,_msgs):
        self.msg = _msgs

    def publish(self,msgs):
        self._pub.publish(msgs)


    def execute_cb(self, goal):
        self.goal = goal
        return goal.result


class CmdControll:
    def __init__(self):
        self.move_vel_sub = RosMaker("sub","/move_vel",MoveMsgs)
        # self.battery_sub = RosMaker("sub","/battery",String)
        self.emergency_sub = RosMaker("sub","/emergency_stop",String)
        self.mode_sub = RosMaker("sub","/robot_mode",String)
        self.charging_sub = RosMaker("sub","/charging",Bool)
        self.move_base_status = RosMaker("sub","/move_base/status",GoalStatusArray)

        self.ai_move_act_srv = RosMaker("action","/ai_move",MoveBaseActAction)
        
        self.cmd_vel_pub = RosMaker("pub","/cmd_vel",Twist)

    def robot_move(self):
        robot_mode_flag = ''
        time_start_flag = False
        while not rospy.is_shutdown() :
            rospy.sleep(0.01)
            twist = Twist()

            emergency_msg = self.emergency_sub.msg.data
            robot_mode = self.mode_sub.msg.data
            self.move_vel_sub.msg
            self.ai_move_act_srv.goal
            
            # if emergency_msg != None or robot_mode != None or move_vel_msg != None or ai_move_msg != None :
            #     emergency_msg = String()
            #     robot_mode = String()
            #     move_vel_msg = MoveMsgs()
            #     ai_move_msg = MoveBaseActGoal()
            #     continue

            os.system("clear")

            # print("ready")
            # print("emergency_msg", emergency_msg)
            # print("robot_mode", robot_mode)
            # print("move_vel_msg", self.move_vel_sub.msg)
            # print("ai_move_msg", self.ai_move_act_srv.goal)

            # print("--"*100)
            # print(self.move_base_status.msg.status_list[0].status)

            if "stop" in emergency_msg and "charging" != robot_mode: #emergency_stop
                # print(self.move_base_status.msg.status_list)
                # if [] != self.move_base_status.msg.status_list :
                #     if 1 != self.move_base_status.msg.status_list[0].status :
                #         twist.linear.x = 0
                #         twist.angular.z = 0
                #         self.cmd_vel_pub.publish(twist)
                #         print("stop")
                # else :
                print("stop_in")
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)
                if self.move_vel_sub.msg.linear_x != 0 and not('emergency' in emergency_msg) :
                    # os.system("mplayer ~/voice/move_away.mp3")
                    print("stop_voice")
                while "stop" in emergency_msg and "charging" != robot_mode:
                    emergency_msg = self.emergency_sub.msg.data
                    robot_mode = self.mode_sub.msg.data
                    twist.linear.x = 0
                    twist.angular.z = 0
                    self.cmd_vel_pub.publish(twist)
                    rospy.sleep(0.1)
                    print("stop")

            elif 'charging' == robot_mode : #charge_go
                print("charging")
                if robot_mode_flag != 'charging' :
                    twist.linear.x = 0
                    twist.angular.z = 0
                    self.cmd_vel_pub.publish(twist)
                    self.move_vel_sub.msg = MoveMsgs()
                    robot_mode_flag = 'charging'
                    print("charging1111")

                if [] != self.move_base_status.msg.status_list :
                    if 1 != self.move_base_status.msg.status_list[0].status :
                        if self.move_vel_sub.msg.header.frame_id == "charging" :
                            twist.linear.x = self.move_vel_sub.msg.linear_x
                            twist.angular.z = self.move_vel_sub.msg.angular_z
                            self.cmd_vel_pub.publish(twist)
                            print(self.move_base_status.msg.status_list[0].status)
                            print("charging22222")


                # if self.ai_move_act_srv.goal.header.frame_id == "charging":
                #     self.movebase_client(self.ai_move_act_srv.goal.x,self.ai_move_act_srv.goal.y,self.ai_move_act_srv.goal.z)
                #     self.ai_move_act_srv.goal = MoveBaseActGoal()



                

            elif 'full_coverage' == robot_mode : #full_coverage
                if robot_mode_flag != 'full_coverage' :
                    twist.linear.x = 0
                    twist.angular.z = 0
                    self.cmd_vel_pub.publish(twist)
                    self.move_vel_sub.msg = MoveMsgs()
                    robot_mode_flag = 'full_coverage'

                # if self.ai_move_act_srv.goal.header.frame_id == 'full_coverage':
                #     self.movebase_client(self.ai_move_act_srv.goal.x,self.ai_move_act_srv.goal.y,self.ai_move_act_srv.goal.z)
                #     self.ai_move_act_srv.goal = MoveBaseActGoal()
                #     print("full_coverage_ai_move")

                # if self.move_vel_sub.msg.header.frame_id == "full_coverage" :
                #     twist.linear.x = self.move_vel_sub.msg.linear_x
                #     twist.angular.z = self.move_vel_sub.msg.angular_z
                    
                #     self.cmd_vel_pub.publish(twist)
                #     print("full_coverage_move_vel")

            elif 'air_condition' == robot_mode : #air_condition
                print("air_condition")
                print(self.ai_move_act_srv.goal)
                if robot_mode_flag != 'air_condition' :
                    robot_mode_flag = 'air_condition'
                    self.move_vel_sub.msg = MoveMsgs()

                # if self.ai_move_act_srv.goal.header.frame_id == 'air_condition':
                #     self.movebase_client(self.ai_move_act_srv.goal.x,self.ai_move_act_srv.goal.y,self.ai_move_act_srv.goal.z)
                #     self.ai_move_act_srv.goal = MoveBaseActGoal()
                #     print("air_conditioning____")


    def movebase_client(self,x,y,z):
        clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
        clear_costmaps_srv()

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = z

        client.send_goal(goal)

        return client.get_result()


def main() :
    rospy.init_node("cmd_controller")
    cmd_controll = CmdControll()
    cmd_controll.robot_move()
    rospy.spin
        
if __name__ == "__main__":
    main()