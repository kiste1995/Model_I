#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import json
import os





def sub_path(msg) :
    cnt = 0

    file_name = "/home/zetabank/catkin_ws/src/zetabot_main/path/full_path_V"+str(cnt)+".json"

    path_dict_list = []
    del_index = []
     

    for i in msg.poses :
        path_dict = {
            "position" : {
                "x" : 0,
                "y" : 0
            },
            "orientation" : {
                "x" : 0,
                "y" : 0,
                "z" : 0,
                "x" : 0
            }
        }
        path_dict["position"]["x"] = i.pose.position.x
        path_dict["position"]["y"] = i.pose.position.y
        path_dict["orientation"]["x"] = i.pose.orientation.x
        path_dict["orientation"]["y"] = i.pose.orientation.y
        path_dict["orientation"]["z"] = i.pose.orientation.z
        path_dict["orientation"]["w"] = i.pose.orientation.w
        path_dict_list.append(path_dict)

    path_dict_list

    for i in range(len(path_dict_list)-1):
        j = i + 1
        if path_dict_list[i] == path_dict_list[j] :
            del_index.append(j)
    
    for i in reversed(del_index) : 
        del(path_dict_list[i])

    while os.path.isfile(file_name) :
        cnt += 1
        file_name = "/home/zetabank/catkin_ws/src/zetabot_main/path/path_V"+str(cnt)+".json"

    with open(file_name, 'w') as full_path_file:
        json.dump(path_dict_list, full_path_file, indent=4)
        
    print(file_name)





def main() :
    rospy.init_node("sub_full_path")
    rospy.sleep(0.5)

    rospy.Subscriber("/move_base/SpiralSTC/plan",Path,sub_path)

    rospy.spin()


if __name__ == "__main__":
    main()