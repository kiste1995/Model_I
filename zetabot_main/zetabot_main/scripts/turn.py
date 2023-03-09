#!/usr/bin/env python
import rospy
import math
import time

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from zetabot_main.srv import TurnSrv, TurnQuaternionSrv


a_val = 0.3
robot_z = 0
pose = Pose()


cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
angle_pub = rospy.Publisher('/angle',Int16,queue_size=10)


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
    global pose
    global robot_z

    pose = val
    X, Y, Z = quaternion_to_euler_angle(pose.orientation)
    robot_z_comp = g_rangle_range_tr(Z)
    robot_z = angle_scailing(robot_z_comp)
    angle_pub.publish(robot_z)


def get_rotation_flag(direction) :
    global robot_z

    relative_angle = direction - robot_z

    if relative_angle < 0 :
        relative_angle =  relative_angle + 360

    if relative_angle <= 3 or relative_angle >= 357 :
        return 0 , relative_angle

    elif relative_angle <= 180 :
        return 1 , relative_angle

    elif relative_angle > 180 :
        return -1 , relative_angle


def turn(comm) :
    global pose
    global robot_z
    global a_val

    roll = 0

    goal_direction = comm.degree

    rospy.sleep(1)
    print('turn_',goal_direction)


    robot_z_original = robot_z



    # goal_direction = cur_direction + (roll) * degree

    # if goal_direction<0 :
    #     goal_direction = goal_direction + 360
    # elif goal_direction >= 360 :
    #     goal_direction = goal_direction - 360

    rotation_flag , relative_angle = get_rotation_flag(goal_direction)
    twist = Twist()
    
    while abs(goal_direction-robot_z)>=3 and rotation_flag != 0 :
        rospy.sleep(0.01)
        twist.angular.z = a_val * rotation_flag
        cmd_vel_pub.publish(twist)
    
    twist = Twist()

    cmd_vel_pub.publish(twist)

    rospy.sleep(0.2)

    return True

def turn_quaternion(comm) :
    X, Y, Z = quaternion_to_euler_angle(comm.orientation)
    z_comp = g_rangle_range_tr(Z)
    z = angle_scailing(z_comp)
    class z_class :
        degree = z
    return turn(z_class)

def main():

    rospy.init_node("turn_test")


    robot_pose_topic = "/robot_pose"
    rospy.Subscriber(robot_pose_topic,Pose,pose_send)

    rospy.sleep(1)

    turn_srv = rospy.Service('/turn', TurnSrv, turn)

    turn_quaternion_srv = rospy.Service('/turn/quaternion', TurnQuaternionSrv, turn_quaternion)

    print("turn_ready")

    rospy.spin()
    

if __name__ == "__main__":
    main()