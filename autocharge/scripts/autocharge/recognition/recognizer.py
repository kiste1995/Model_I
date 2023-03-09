from __future__ import print_function

import os, sys

sys.path.append(os.path.abspath(os.path.dirname(__file__)))


import numpy as np
import cv2
import cgitb
import math
import re
from time import time
from time import sleep

from log.logger import Logger
from deco import recognizer

cgitb.enable(format= 'text')

@recognizer
class Recongnition:
    LOG = Logger()

    def __init__(self):
        self.arr = []
        self.image_processing_finish_flag = False
        self.video_logger_flag = False
        self.thd_end = False

        self.bCharing = False

        self.frame_monitor = self.frame_monitor
        self.blurred_frame_monitor = self.blurred_frame_monitor
        self.hsv_merge_monitor = self.hsv_merge_monitor
        self.IR_camera_monitor = self.IR_camera_monitor
        self.gray_frame_monitor = self.gray_frame_monitor
        self.binary_camera_monitor = self.binary_camera_monitor
        self.contours_monitor = self.contours_monitor
        self.rectangle_monitor = self.rectangle_monitor

        self.cam_cap = []

    def video_detect_index(self):
        video_idx = None

        for video_idx_file in os.listdir("/sys/class/video4linux"):
            video_file_path = os.path.realpath("/sys/class/video4linux/" + video_idx_file)

            for video_files in os.listdir(video_file_path):
                if 'name' in video_files:
                    name = open(os.path.realpath(video_file_path + '/' + 'name'), 'r').readline()
                    print("name:" + name)

                    # if name.find("Video Capture 2") > -1:
                    #     index = open(os.path.realpath(video_file_path + '/' + 'index'), 'r').readline()
                    #     print("index" + index)

                    #     video_idx = "/dev/" + video_idx_file
                    #     print("video_idx:" + video_idx)

                    #     return video_idx

                    if re.match('KINGSEN', name) != None:
                        index = open(os.path.realpath(video_file_path + '/' + 'index'), 'r').readline()
                        if re.match('0', index) != None:
                            video_idx = "/dev/" + video_idx_file
                            print(video_idx)

        return video_idx

    def preprocess_image(self, _frame):
        blurred_frame = cv2.GaussianBlur(_frame, ksize= (3, 3), sigmaX= 0, sigmaY= 0)

        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit= 1.3, tileGridSize= (5, 5))
        clahe_s = clahe.apply(s)
        hsv_merge = cv2.merge((h, clahe_s, v))
        #cvt_frame = cv2.cvtColor(hsv_merge, cv2.COLOR_HSV2BGR)

        ir_mark = cv2.inRange(hsv_merge, \
            (self.ir_maker_lower_h, self.ir_maker_lower_s, self.ir_maker_lower_v), \
            (self.ir_maker_upper_h, self.ir_maker_upper_s, self.ir_maker_upper_v))
        ir_frame = cv2.bitwise_and(hsv, hsv, mask= ir_mark)
        ir_frame = cv2.cvtColor(ir_frame, cv2.COLOR_HSV2BGR)

        gray_frame = cv2.cvtColor(ir_frame, cv2.COLOR_BGR2GRAY)
        _, binary_frame = cv2.threshold(gray_frame, 0, self.binary_threshold, cv2.THRESH_BINARY)

        kernel = np.ones((3,3), np.uint8)
        dilate_frame = cv2.dilate(binary_frame, kernel, iterations = self.dilate_iterations)
        kernel = np.ones((2,2), np.uint8)
        erode_frame = cv2.erode(dilate_frame, kernel, iterations = self.erode_iterations)

        #contours, _ = cv2.findContours(erode_frame, mode= cv2.RETR_EXTERNAL, method= cv2.CHAIN_APPROX_SIMPLE)
        _, contours, _ = cv2.findContours(erode_frame, mode= cv2.RETR_EXTERNAL, method= cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            if 250 < area:
                cv2.rectangle(erode_frame, pt1 = (x, y), pt2 = (x+w, y+h), color = (230, 255, 100), thickness = -1)

        #contours, _ = cv2.findContours(erode_frame, mode= cv2.RETR_EXTERNAL, method= cv2.CHAIN_APPROX_SIMPLE)
        _, contours, _ = cv2.findContours(erode_frame, mode= cv2.RETR_EXTERNAL, method= cv2.CHAIN_APPROX_SIMPLE)

        result = np.zeros((480, 640, 3), dtype= np.uint8)
        cv2.drawContours(result, contours= contours, contourIdx= -1, color= (255, 255, 255), thickness= 0)

        if self.frame_monitor: cv2.imshow("frame", _frame)
        if self.blurred_frame_monitor: cv2.imshow("blurred_frame", blurred_frame)
        if self.hsv_merge_monitor: cv2.imshow("hsv_merge", hsv_merge)
        if self.IR_camera_monitor: cv2.imshow("IR_camera", ir_frame)
        if self.gray_frame_monitor: cv2.imshow("gray_frame", gray_frame)
        if self.binary_camera_monitor: cv2.imshow("binary_camera", binary_frame)
        if self.contours_monitor: cv2.imshow("contours", result)

        cv2.waitKey(1)

        return result, contours

    def select_ROI(self, _preprocessed_image, _contours):
        ROI_contours = []

        for contour in _contours:
            x, y, w, h = cv2.boundingRect(contour)
            y_center_point = y + (w/2)
            x_center_point = x + (h/2)
            ratio = round(float(w)/float(h), 1)

            if self.min_width < w < self.max_width and self.min_height < h < self.max_height and self.min_ratio < ratio < self.max_ratio and self.min_point < y_center_point < self.max_point:
                cv2.rectangle(_preprocessed_image, pt1 = (x, y), pt2 = (x+w, y+h), color = (100, 255, 100), thickness = 1)

                ROI_contours.append({
                    #'x_cp' : x_center_point, 'y_cp' : y_center_point, 'w' : w, 'h' : h, 'x' : x, 'y' : y, 'contour' : contour
                    'x_cp': x_center_point, 'y_cp': y_center_point, 'x': x, 'y': y, 'w': w, 'h': h, 'area': w*h, 'w/h ratio': ratio
                })
                #print('ROI_contours:', ROI_contours)

        if self.rectangle_monitor: cv2.imshow("rectangle", _preprocessed_image)

        return ROI_contours

    def first_match_contours(self, _ROI_contoures):
        first_marker = []
        second_marker = []
        third_marker = []
        fifth_marker = []

        angle = 0

        result_0 = np.zeros((480, 640, 3), dtype= np.uint8)

        if len(_ROI_contoures) == 0:
            #print("not exist markers!")
            pass

        else:
            for i in _ROI_contoures:
                first_marker = []
                third_marker = []
                fifth_marker = []

                for j in _ROI_contoures:
                    if i['x_cp'] == j['x_cp'] and i['y_cp'] == j['y_cp']: continue

                    area_diff = round(abs(i['area'] - j['area']) / i['area'], 2)
                    if area_diff > 0.5: continue

                    x_derivative = round(i['x_cp'] - j['x_cp'], 4)
                    y_derivative = round(i['y_cp'] - j['y_cp'], 4)
                    diff_x_value = abs(x_derivative)
                    diff_y_value = abs(y_derivative)

                    if diff_y_value == 0:
                        if x_derivative < 0: angle = -180
                        elif x_derivative > 0: angle = 0
                    elif diff_x_value == 0:
                        if y_derivative < 0: angle = -90
                        elif y_derivative > 0: angle = 90
                    else:
                        angle = round((np.arctan2(y_derivative, x_derivative) * 180) / np.pi, 0)

                    if angle <= -150 or angle >= 150:
                        if diff_y_value < 20:
                            if x_derivative < 0:
                                third_marker = j
                    elif -120 <= angle <= -60:
                        if diff_x_value < 20:
                            if y_derivative < 0:
                                fifth_marker = j
                    elif -30 <= angle <= 30:
                        if diff_y_value < 20:
                            if x_derivative > 0:
                                first_marker = j

                if first_marker != [] and third_marker != [] and fifth_marker != []:
                    second_marker = i

                    #cv2.rectangle(result_0, pt1 = (second_marker['x'], second_marker['y']), pt2 = (second_marker['x'] + second_marker['w'], second_marker['y'] + second_marker['h']), color = (255, 255, 100), thickness = 1)
                    #cv2.rectangle(result_0, pt1 = (first_marker['x'], first_marker['y']), pt2 = (first_marker['x'] + first_marker['w'], first_marker['y'] + first_marker['h']), color = (0, 255, 100), thickness = 1)
                    #cv2.rectangle(result_0, pt1 = (third_marker['x'], third_marker['y']), pt2 = (third_marker['x'] + third_marker['w'], third_marker['y'] + third_marker['h']), color = (0, 255, 100), thickness = 1)
                    #cv2.rectangle(result_0, pt1 = (fifth_marker['x'], fifth_marker['y']), pt2 = (fifth_marker['x'] + fifth_marker['w'], fifth_marker['y'] + fifth_marker['h']), color = (0, 255, 100), thickness = 1)

                    break

        #cv2.imshow("result_0", result_0)

        return first_marker, second_marker, third_marker

    def second_match_contours(self, _ROI_contoures):
        second_marker = []
        fourth_marker = []
        fifth_marker = []
        sixth_marker = []

        angle = 0

        result_1 = np.zeros((480, 640, 3), dtype= np.uint8)

        if len(_ROI_contoures) == 0:
            #print("not exist markers!")
            pass

        else:
            for i in _ROI_contoures:
                second_marker = []
                fourth_marker = []
                sixth_marker = []

                for j in _ROI_contoures:
                    if i['x_cp'] == j['x_cp'] and i['y_cp'] == j['y_cp']: continue

                    area_diff = round(abs(i['area'] - j['area']) / i['area'], 2)
                    if area_diff > 0.5: continue

                    x_derivative = round(i['x_cp'] - j['x_cp'], 4)
                    y_derivative = round(i['y_cp'] - j['y_cp'], 4)
                    diff_x_value = abs(x_derivative)
                    diff_y_value = abs(y_derivative)

                    if diff_y_value == 0:
                        if x_derivative < 0: angle = -180
                        elif x_derivative > 0: angle = 0
                    elif diff_x_value == 0:
                        if y_derivative < 0: angle = -90
                        elif y_derivative > 0: angle = 90
                    else:
                        angle = round((np.arctan2(y_derivative, x_derivative) * 180) / np.pi, 0)

                    if angle <= -150 or angle >= 150:
                        if diff_y_value < 20:
                            if x_derivative < 0:
                                sixth_marker = j
                    elif -30 <= angle <= 30:
                        if diff_y_value < 20:
                            if x_derivative > 0:
                                fourth_marker = j
                    elif 60 <= angle <= 120:
                        if diff_x_value < 20:
                            if y_derivative > 0:
                                second_marker = j

                if second_marker != [] and fourth_marker != [] and sixth_marker != []:
                    fifth_marker = i

                    #cv2.rectangle(result_1, pt1 = (fifth_marker['x'], fifth_marker['y']), pt2 = (fifth_marker['x'] + fifth_marker['w'], fifth_marker['y'] + fifth_marker['h']), color = (255, 255, 100), thickness = 1)
                    #cv2.rectangle(result_1, pt1 = (second_marker['x'], second_marker['y']), pt2 = (second_marker['x'] + second_marker['w'], second_marker['y'] + second_marker['h']), color = (0, 255, 100), thickness = 1)
                    #cv2.rectangle(result_1, pt1 = (fourth_marker['x'], fourth_marker['y']), pt2 = (fourth_marker['x'] + fourth_marker['w'], fourth_marker['y'] + fourth_marker['h']), color = (0, 255, 100), thickness = 1)
                    #cv2.rectangle(result_1, pt1 = (sixth_marker['x'], sixth_marker['y']), pt2 = (sixth_marker['x'] + sixth_marker['w'], sixth_marker['y'] + sixth_marker['h']), color = (0, 255, 100), thickness = 1)

                    break

        #cv2.imshow("result_1", result_1)

        return fourth_marker, fifth_marker, sixth_marker

    def third_match_contours(self, final_result, first_marker, second_marker, third_marker, fourth_marker, fifth_marker, sixth_marker):
        _matched_contours = []

        if first_marker == [] or second_marker == [] or third_marker == [] or fourth_marker == [] or fifth_marker == [] or sixth_marker == []:
            final_result = np.zeros((480, 640, 3), dtype= np.uint8)
            #print("empty marker!!!!")

        else: 
            diff_x_value_0 = abs(first_marker['x_cp'] - fourth_marker['x_cp'])
            diff_x_value_1 = abs(third_marker['x_cp'] - sixth_marker['x_cp'])

            if diff_x_value_0 < 10:
                if diff_x_value_1 < 10:
                    final_result = np.zeros((480, 640, 3), dtype= np.uint8)

                    cv2.rectangle(final_result, pt1 = (first_marker['x'], first_marker['y']), pt2 = (first_marker['x'] + first_marker['w'], first_marker['y'] + first_marker['h']), color = (200, 200, 200), thickness = 1)
                    cv2.rectangle(final_result, pt1 = (second_marker['x'], second_marker['y']), pt2 = (second_marker['x'] + second_marker['w'], second_marker['y'] + second_marker['h']), color = (200, 200, 200), thickness = 1)
                    cv2.rectangle(final_result, pt1 = (third_marker['x'], third_marker['y']), pt2 = (third_marker['x'] + third_marker['w'], third_marker['y'] + third_marker['h']), color = (200, 200, 200), thickness = 1)
                    cv2.rectangle(final_result, pt1 = (fourth_marker['x'], fourth_marker['y']), pt2 = (fourth_marker['x'] + fourth_marker['w'], fourth_marker['y'] + fourth_marker['h']), color = (200, 200, 200), thickness = 1)
                    cv2.rectangle(final_result, pt1 = (fifth_marker['x'], fifth_marker['y']), pt2 = (fifth_marker['x'] + fifth_marker['w'], fifth_marker['y'] + fifth_marker['h']), color = (200, 200, 200), thickness = 1)
                    cv2.rectangle(final_result, pt1 = (sixth_marker['x'], sixth_marker['y']), pt2 = (sixth_marker['x'] + sixth_marker['w'], sixth_marker['y'] + sixth_marker['h']), color = (200, 200, 200), thickness = 1)

                    _matched_contours.append(first_marker)
                    _matched_contours.append(second_marker)
                    _matched_contours.append(third_marker)
                    _matched_contours.append(fourth_marker)
                    _matched_contours.append(fifth_marker)
                    _matched_contours.append(sixth_marker)

        if self.result_monitor: cv2.imshow("result_monitor", final_result)

        return _matched_contours

    def calculation_position(self, _mateched_contours):
        _diff_ratio = 0.0
        _area_ratio = 0.0
        _left_area = 0
        _right_area = 0
        _total_distance = 0
        _total_area = 0
        _center_check = "-"
        _robot_position = "-"

        if _mateched_contours == []:
            return _total_distance, _total_area, _diff_ratio, _center_check, _robot_position

        marker_center_point_x = (_mateched_contours[1]['x_cp'] + _mateched_contours[4]['x_cp'])/2
        
        if (320 - self.marker_center_margin) < marker_center_point_x < (320 + self.marker_center_margin):
            _center_check = 'CENTER'
        elif marker_center_point_x <= (320 - self.marker_center_margin):
            _center_check = 'LEFT'
        elif marker_center_point_x >= (320 + self.marker_center_margin):
            _center_check = 'RIGHT'
        elif marker_center_point_x == 0:
            _center_check = '-'
        else:
            _center_check = '-'

        #print('marker_center_point_x: ', marker_center_point_x)
        #print('_center_check: ', _center_check)

        _left_distance = _mateched_contours[1]['x_cp'] - _mateched_contours[0]['x_cp']
        _right_distance = _mateched_contours[2]['x_cp'] - _mateched_contours[1]['x_cp']
        _total_distance = _left_distance + _right_distance
        _left_area = (_left_distance) * (_mateched_contours[3]['y_cp'] - _mateched_contours[0]['y_cp'])
        _right_area = (_right_distance) * (_mateched_contours[5]['y_cp'] - _mateched_contours[2]['y_cp'])
        _total_area = _left_area + _right_area

        if _left_area > _right_area:
            _diff_ratio = round(float(_right_area)/float(_left_area), 2)
            if (_left_distance - _right_distance) <= self.robot_center_margin:
                _robot_position = 'CENTER'
            else:
                _robot_position = 'LEFT'

        elif _right_area > _left_area:
            _diff_ratio = round(float(_left_area)/float(_right_area), 2)
            if (_right_distance - _left_distance) <= self.robot_center_margin:
                _robot_position = 'CENTER'
            else:
                _robot_position = 'RIGHT'

        else:
            _diff_ratio = 1
            _robot_position = 'CENTER'
 
        #print("_center_check: ", _center_check)
        #print("_robot_position: ", _robot_position)
        #print("total_area: %d, diff_ratio: %f" %(_total_area, _diff_ratio))
        #print("left_area: %d, right_area: %d, total_area: %d, diff_ratio: %f" %(_left_area, _right_area, _total_area, _diff_ratio))

        return _total_distance, _total_area, _diff_ratio, _center_check, _robot_position

    def calculation_distance(self, _total_distance, _total_area, _diff_ratio):
        _degree = 0.0
        _distance = 0.0
        _target_distance = 0.0

        if _total_area == 0:
            return _degree, _target_distance

        _degree = round(-36.83 * (_diff_ratio * _diff_ratio) + 118.45 * _diff_ratio + 8.28, 2)
        _distance = round(12222 * math.pow((_total_area + 500), -0.543), 2)

        radian = math.radians(_degree)
        _target_distance = round(_distance * (1/(math.tan(radian))), 2)

        #print("_degree: %f, _distance: %f, _target_distance: %f" %(_degree, _distance, _target_distance))

        return _degree, _target_distance

    def image_processing(self, cap):

        self.cam_cap = cap
        # cap = cv2.VideoCapture(self.video_detect_index())

        # print("detect video index:" + self.video_detect_index())

        # if not cap.isOpened():
        #     print("camera open failed!")
        #     #sys.exit()

        self.caperr_cnt = 0

        self.cam_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        fps = self.cam_cap.get(cv2.CAP_PROP_FPS)

        final_result = np.zeros((480, 640, 3), dtype= np.uint8)

        self.LOG.video_log_start(fps)

        try:
            delay = int(600/fps)
        except ZeroDivisionError:
            print("ZeroDivisionError")
            delay = 35

        # print("image_processing [1]")

        while True:

            # print("image_processing [2]")

            if self.image_processing_finish_flag == True:
                self.LOG.video_log_close()
                self.image_processing_finish_flag = False

                print("finish proc image processing...")

            # if self.bCharing == True:
                try:
                    # cap.release()
                    cv2.destroyAllWindows()
                    # cap2.release()
                except:
                    pass
                break

            # if self.bCharing == True:
            #     print("Exit thread image processing...")
            #     break

            try:
                ret, frame = self.cam_cap.read()

                # print("image_processing [3]")

                if ret:
                    # print("image_processing [4]")

                    self.LOG.video_logging(frame)

                    # print("image_processing [5]")

                    #
                    preprocessed_image, contours = self.preprocess_image(frame)
                    ROI_contours = self.select_ROI(preprocessed_image, contours)

                    #
                    first_marker, second_marker, third_marker = self.first_match_contours(ROI_contours)
                    fourth_marker, fifth_marker, sixth_marker = self.second_match_contours(ROI_contours)
                    matched_contours = self.third_match_contours(final_result, first_marker, second_marker, third_marker, fourth_marker, fifth_marker, sixth_marker)

                    #
                    total_distance, total_area, diff_ratio, center_check, robot_position = self.calculation_position(matched_contours)
                    degree, target_distance = self.calculation_distance(total_distance, total_area, diff_ratio)
                    
                    self.arr = [center_check, robot_position, degree, target_distance]
                    #print("sub:", self.arr)

                    # print("image_processing [6]")
                else:
                    print("can't open video!!")

                    self.caperr_cnt += 1

                    if self.caperr_cnt > 10:
                        self.caperr_cnt = 0

                        self.reOpenCamDevice()

                    cv2.destroyAllWindows()   
            except:
                print("video capture error...")      

                self.caperr_cnt += 1

                if self.caperr_cnt > 10:
                        self.caperr_cnt = 0

                        self.reOpenCamDevice()

                # print("image_processing [7]")      

            if self.thd_end == True:
                self.thd_end = False

                print("image_processing : the_end")

                # print("image_processing [8]")
                break

            sleep(0.05)

            # print("image_processing [9]")
        
            
            #if cv2.waitKey(delay + 3) == ord('q'):
            #    self.LOG.video_log_close()
            #    cv2.destroyAllWindows()
            #    break 

        # print("image_processing [10]")
        print("Exit image processing...")

    def AllWindowsClose(self):
        cv2.destroyAllWindows()
        print("Close all windows...")

    def reOpenCamDevice(self):
        vindex = self.video_detect_index()
        # print("video device index : ")
        # print(vindex)

        self.cam_cap = cv2.VideoCapture(vindex)

        print("ReOpen CAM Device ==> detect video index:" + str(vindex))