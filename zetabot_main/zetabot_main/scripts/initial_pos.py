#!/usr/bin/env python
import rospy
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from zetabot_main.srv import InitPoseSrv


def initial_pos_pub(msg):

    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    #Creating the message with the type PoseWithCovarianceStamped
    rospy.loginfo("This node sets the turtlebot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
    start_pos = PoseWithCovarianceStamped()
    #filling header with relevant information
    start_pos.header.frame_id = "map"
    start_pos.header.stamp = rospy.Time.now()
    #filling payload with relevant information gathered from subscribing
    # to initialpose topic published by RVIZ via rostopic echo initialpose
    start_pos.pose.pose.position.x = msg.position_x
    start_pos.pose.pose.position.y = msg.position_y
    start_pos.pose.pose.position.z = 1.0

    start_pos.pose.pose.orientation.x = 0.0
    start_pos.pose.pose.orientation.y = 0.0
    start_pos.pose.pose.orientation.z = msg.orientation_z
    start_pos.pose.pose.orientation.w = msg.orientation_w

    start_pos.pose.covariance[0] = 0.25
    start_pos.pose.covariance[7] = 0.25
    start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[35] = 0.06853891945200942

    rospy.loginfo(start_pos)
    rospy.sleep(1)
    publisher.publish(start_pos)
    rospy.sleep(1)

    return True


def main():

    rospy.init_node("init_pose_srv")


    start_pose_param = rospy.get_param("start_pose")

    start_pose = InitPoseSrv()
    start_pose.position_x = start_pose_param['position_x']
    start_pose.position_y = start_pose_param['position_y']
    start_pose.orientation_z = start_pose_param['orientation_z']
    start_pose.orientation_w = start_pose_param['orientation_w']

    rospy.sleep(1)

    initial_pos_pub(start_pose)


    rospy.sleep(1)

    srv = rospy.Service('/init_pose_srv', InitPoseSrv, initial_pos_pub)

    print("init_pose_srv_ready")

    rospy.spin()
    

if __name__ == "__main__":
    main()