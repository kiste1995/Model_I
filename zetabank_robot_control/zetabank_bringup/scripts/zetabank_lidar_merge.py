#!/usr/bin/env python
import numpy
import math
import rospy
from sensor_msgs.msg import LaserScan

LaserScanMsg = LaserScan()
range_pre = [[],[]]


def scanCB(scan):
    global LaserScanMsg
    LaserScanMsg.time_increment = scan.time_increment
    LaserScanMsg.angle_increment = scan.angle_increment
    LaserScanMsg.header.stamp = scan.header.stamp
    LaserScanMsg.scan_time = scan.scan_time
    lidar_merge_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    if "/laser_1" in scan.header.frame_id:
        for (index,range) in enumerate(scan.ranges):
            if range < range_pre[1][index]:
                LaserScanMsg.ranges[index] = range
                LaserScanMsg.intensities[index] = scan.intensities[index]
        range_pre[0] = scan.ranges
    elif "/laser_2" in scan.header.frame_id:
        for (index,range) in enumerate(scan.ranges):
            if range < range_pre[0][index]:
                LaserScanMsg.ranges[index] = range
                LaserScanMsg.intensities[index] = scan.intensities[index]
        range_pre[1] = scan.ranges
    lidar_merge_pub.publish(LaserScanMsg)


def initLaserScan():
    global LaserScanMsg
    LaserScanMsg.header.frame_id = "/laser_merge"
    LaserScanMsg.range_max = 100.0
    LaserScanMsg.range_min = 0.05
    LaserScanMsg.angle_max = 2.35619449615
    LaserScanMsg.angle_min = -2.35619449615
    LaserScanMsg.ranges = [5.0]*271
    LaserScanMsg.intensities = [0.0]*271
    range_pre[0] = [5.0]*271
    range_pre[1] = [5.0]*271
    

if __name__ == "__main__":
    rospy.init_node('lidar_merge')
    initLaserScan()
    scan = list()
    lidar = list()
    scan.append("/scan_1") 
    scan.append("/scan_2")
    lidar.append(rospy.Subscriber(scan[0], LaserScan, scanCB))
    lidar.append(rospy.Subscriber(scan[1], LaserScan, scanCB))
    rospy.spin()


#EOF