import cv2
import csv
import time
import os
import datetime

from deco import logger

@logger
class Logger:
    def __init__(self):
        self.sequence_logger_enable = self.sequence_logger_enable
        self.video_logger_enable = self.video_logger_enable
        
        self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')

    def sequence_logger(self, sequence):
        if self.sequence_logger_enable:
            today = time.strftime('%m.%d', time.localtime(time.time()))
            f = open(self.sequence_log_save_dir + today + ".csv", 'a')
            wr = csv.writer(f)
            log_content = [time.strftime('%H:%M:%S', time.localtime(time.time())), sequence]
            wr.writerow(log_content)
            f.close()
        else:
            print("sequence_logger_disabled")

    def sequence_log_remover(self):
        pass

    def video_log_start(self, fps):
        if self.video_logger_enable:
            print("video record start")
            self.video_log_remover()

            _today = time.strftime('%m_%d', time.localtime(time.time()))
            _time = time.strftime('%H_%M_%S', time.localtime(time.time()))

            self.create_folder(self.video_save_folder_dir + _today)
            self.out = cv2.VideoWriter(self.video_save_folder_dir + _today + "/" + _time + ".avi", self.fourcc, fps, (640, 480))
        else:
            print("video_logger_disabled")

    def video_logging(self, frame) :
        if self.video_logger_enable:
            cv2.putText(frame, text=time.strftime('%m-%d %H:%M:%S', time.localtime(time.time())), org=(30, 450), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,255,0), thickness=2)
            self.out.write(frame)
        else:
            pass

    def video_log_close(self):
        if self.video_logger_enable:
            print("video record end")
            self.out.release()
        else:
            print("video_logger_disabled")

    def video_log_remover(self):
        keep_flag = False
        keeping_time_month_list = []
        keeping_time_day_list = []

        #now_time = datetime.datetime.now().strftime('%m_%d_%H_%M')
        #print("now_time:", now_time)

        for i in range(self.keeping_days):
            keeping_time = (datetime.datetime.now() - datetime.timedelta(days= i)).strftime('%m-%d')

            keeping_time_list = keeping_time.split('-')
            keeping_time_month_list.append(int(keeping_time_list[0]))
            keeping_time_day_list.append(int(keeping_time_list[1]))

            #print('keeping_time_month_list', keeping_time_month_list)
            #print('keeping_time_day_list', keeping_time_day_list)

        for video_file in os.listdir(self.video_save_folder_dir):
            video_file_created_time = video_file.split('_')
            video_file_created_time_month = int(video_file_created_time[0])
            video_file_created_time_day = int(video_file_created_time[1])

            for keeping_day in keeping_time_day_list:
                if video_file_created_time_day == keeping_day:
                    keep_flag = True
                
            if keep_flag == True:
                keep_flag = False
                #print('video_file_created_time_month', video_file_created_time_month)
                #print('video_file_created_time_day', video_file_created_time_day)

            else:
                remove_folder_name = str(video_file_created_time_month) + "_" + str(video_file_created_time_day)
                remove_path = self.video_save_folder_dir + "/" + remove_folder_name
                os.rmdir(remove_path)

    def create_folder(self, directory):
        try:
            if not os.path.exists(directory):
                os.makedirs(directory)
        except OSError:
            print("Error: Creating directory. " + directory)
