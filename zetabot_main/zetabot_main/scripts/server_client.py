#! /usr/bin/env python

import rospy
# requests import -> pip install requests
import requests

rospy.init_node("server_client")

# rospy.get_param("server_url","")

# url = rospy.get_param("server_url","")
url = "http://13.124.4.101:8000/file/upload/"
files = {'file': open('/home/zetabank/robot_log/air_log/air_log_2021_06_25.csv', 'rb')}
params = {"file":"sensor_0625.csv", "description":"test_1"}
getdata = requests.post(url, params, files=files)
print(getdata.text)