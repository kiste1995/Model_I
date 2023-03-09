import os
import time
import platform
import rospy

root_dir = os.path.dirname(os.path.abspath(__file__))
_ = '\\' if platform.system() == 'Windows' else '/'
if root_dir[len(root_dir) - 1] != _: root_dir += _

#print("root_dir: ", root_dir)

charging_station_pose = rospy.get_param("/charging_station_pose")
robot_init_pose = rospy.get_param("/robot_init_pose")

today = time.strftime('%m_%d_%H_%M_%S', time.localtime(time.time()))

BASE = {
    'root_dir': root_dir.format(_=_)
}

LOGGER = {
    'sequence_logger_enable': True , # enable = True, disable = False
    'video_logger_enable': False , # enable = True, disable = False
    'video_log_remove_7day': False,
    'video_log_remove_15day': False,
    'video_log_remove_30day': False,
    'sequence_log_save_dir': BASE['root_dir'] + "saved/log/autocharge_sequence_log_",
    'video_save_folder_dir': BASE['root_dir'] + "saved/video",

    'keeping_days': 7, # 1 ~ 25days
}

RECOGNIZER = {
    'frame_monitor': False,
    'blurred_frame_monitor': False,
    'hsv_merge_monitor': False,
    'IR_camera_monitor': True,
    'gray_frame_monitor': False,
    'binary_camera_monitor': False,
    'contours_monitor': True,
    'rectangle_monitor': False,
    'result_monitor': True,

    'delay_time': 35,

    'binary_threshold': 255,
    'dilate_iterations': 2,
    'erode_iterations': 2,

    #'ir_maker_lower_h': 0,
    #'ir_maker_lower_s': 0,
    #'ir_maker_lower_v': 0,
    #'ir_maker_upper_h': 179,
    #'ir_maker_upper_s': 255,
    #'ir_maker_upper_v': 255,

    'ir_maker_lower_h': 0,
    'ir_maker_lower_s': 0,
    'ir_maker_lower_v': 230,
    'ir_maker_upper_h': 179,
    'ir_maker_upper_s': 30,
    'ir_maker_upper_v': 255,

    'min_area': 50, 'max_area': 20000,
    'min_width': 5, 'max_width': 150,
    'min_height': 10, 'max_height': 100,
    'min_point': 0, 'max_point': 480,
    'min_ratio': 0.5, 'max_ratio': 1.3,

    'num_matched': 6,
    'max_area_diff': 0.4,

    'marker_center_margin': 3, # 0 ~ 10
    'robot_center_margin': 6 # 0 ~ 10
}

DOCKING = {
    'initial_position_x': robot_init_pose['position_x'],
    'initial_position_y': robot_init_pose['position_y'],
    'initial_orientation_z': robot_init_pose['orientation_z'],
    'initial_orientation_w': robot_init_pose['orientation_w'],

    'charge_position_x': charging_station_pose["position_x"],
    'charge_position_y': charging_station_pose["position_y"],
    'charge_degree': charging_station_pose["degree"]
}

ROS = {
    'initial_position_x': robot_init_pose['position_x'],
    'initial_position_y': robot_init_pose['position_y'],
    'initial_orientation_z': robot_init_pose['orientation_z'],
    'initial_orientation_w': robot_init_pose['orientation_w'],

    'charge_position_x': charging_station_pose["position_x"],
    'charge_position_y': charging_station_pose["position_y"],
    'charge_degree': charging_station_pose["degree"]
}