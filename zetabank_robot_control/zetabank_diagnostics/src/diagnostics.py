#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: Unspecified
#
import rospy
import smach
import time
import argparse
import sys
import os
import json
import struct
import subprocess
from std_msgs.msg import String, Float32, Bool
from can_msgs.msg import Frame
from sensor_msgs.msg import Imu, LaserScan
from zetabot_main.msg import EnvironmentMsgs, BatteryInformationMsgs, SonarArray
from core import init_blackboard, log
import roslaunch
import diagnostic_updater
import diagnostic_msgs

# rest api
import requests
from requests.auth import HTTPBasicAuth

from kafka import KafkaProducer
from kafka.errors import KafkaError

producer = KafkaProducer(
    bootstrap_servers=["15.165.218.44:9093", "3.38.10.119:9094", "3.38.88.134:9095"]
    # naver bootstrap_servers=["221.168.33.178:9093", "221.168.33.178:9094", "221.168.33.178:9095"]
)


class preempted_timeout(smach.State):
    def __init__(self, timeout):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.timeout = timeout
        rospy.loginfo('timeout is {}'.format(self.timeout))

    def execute(self, ud):
        start_time = time.time()
        while True:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            time.sleep(0.2)
            if time.time() - start_time > self.timeout:
                break
        return 'succeeded'


class test_diagnostics(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("zetabot_1102")
        self.updater.add("data aggregation", self.data_aggr_summary)

        self.air = {}
        self.auto_charge_ino_value = ""
        self.battery97 = {}
        self.battery96 = {}
        self.battery_soc = 0.0
        self.emergency_stop = ""
        self.estop = False
        self.imu = {}
        self.sonar = []
        self.bumper = {}
        self.water_level = {}
        self.data_aggr = {}
        self.data = {}
        self.pub_data_aggr = rospy.Publisher('/diagnostic_aggr', String, queue_size=10)

        rospy.Subscriber('/air', EnvironmentMsgs, self.sub_air)
        freq_bounds = {'min': 0.09,
                       'max': 0.2}  # average rate: 0.107 frequency 허용 범위, min, max 허용 공차, 10초 안에 들어온 event 수
        self.air_freq = diagnostic_updater.HeaderlessTopicDiagnostic("air", self.updater,
                                                                     diagnostic_updater.FrequencyStatusParam(
                                                                         freq_bounds,  # 허용 범위(hz)
                                                                         0.05,  # 허용 공차
                                                                         12))  # 10s 안에 발생한 event 수

        rospy.Subscriber('/autocharge_state_INO', String, self.sub_auto_charge_state_ino)
        freq_bounds = {'min': 30, 'max': 35}  # average rate: 34.046
        self.auto_charge_INO_freq = diagnostic_updater.HeaderlessTopicDiagnostic("auto_charge_INO", self.updater,
                                                                                 diagnostic_updater.FrequencyStatusParam(
                                                                                     freq_bounds, 0.01, 1))
        rospy.Subscriber('/battery', BatteryInformationMsgs, self.sub_battery)
        freq_bounds = {'min': 2, 'max': 2.5}  # average rate: 2.268
        self.battery_freq = diagnostic_updater.HeaderlessTopicDiagnostic("battery", self.updater,
                                                                         diagnostic_updater.FrequencyStatusParam(
                                                                             freq_bounds, 0.05, 5))
        rospy.Subscriber('/battery_SOC', Float32, self.sub_battery_soc)
        freq_bounds = {'min': 2, 'max': 2.2}  # average rate: 2.141
        self.battery_freq_soc_freq = diagnostic_updater.HeaderlessTopicDiagnostic("battery_SOC", self.updater,
                                                                                  diagnostic_updater.FrequencyStatusParam(
                                                                                      freq_bounds, 0.01, 5))

        rospy.Subscriber('/emergency_stop', String, self.sub_emergency_stop)
        freq_bounds = {'min': 77, 'max': 85}  # average rate: 77.635
        self.emergency_stop_freq = diagnostic_updater.HeaderlessTopicDiagnostic("emergency_stop", self.updater,
                                                                                diagnostic_updater.FrequencyStatusParam(
                                                                                    freq_bounds, 0.01, 1))

        rospy.Subscriber('/estop', Bool, self.sub_eStop)
        freq_bounds = {'min': 4.5, 'max': 5}  # average rate: 4.919
        self.estop_freq = diagnostic_updater.HeaderlessTopicDiagnostic("estop", self.updater,
                                                                       diagnostic_updater.FrequencyStatusParam(
                                                                           freq_bounds, 0.01, 7))

        rospy.Subscriber('/imu', Imu, self.sub_imu)
        freq_bounds = {'min': 30, 'max': 40}  # average rate: 34.087
        self.imu_freq = diagnostic_updater.HeaderlessTopicDiagnostic("imu", self.updater,
                                                                     diagnostic_updater.FrequencyStatusParam(
                                                                         freq_bounds, 0.01, 1))
        rospy.Subscriber('/scan', LaserScan, self.sub_scan)
        freq_bounds = {'min': 10, 'max': 20}  # average rate: 15.982
        self.scan_freq = diagnostic_updater.HeaderlessTopicDiagnostic("scan", self.updater,
                                                                      diagnostic_updater.FrequencyStatusParam(
                                                                          freq_bounds, 0.01, 1))
        rospy.Subscriber('/sonar', SonarArray, self.sub_sonar)
        freq_bounds = {'min': 10, 'max': 20}  # average rate: 15.441
        self.sonar_freq = diagnostic_updater.HeaderlessTopicDiagnostic("sonar", self.updater,
                                                                       diagnostic_updater.FrequencyStatusParam(
                                                                           freq_bounds, 0.01, 1))

        rospy.Subscriber('/diagnostics', diagnostic_msgs.msg.DiagnosticArray, self.sub_diagnostics)

        rospy.Subscriber('/bumper', Bool, self.sub_bumper)
        freq_bounds = {'min': 4, 'max': 5}  # average rate: 4.919
        self.bumper_freq = diagnostic_updater.HeaderlessTopicDiagnostic("bumper", self.updater,
                                                                        diagnostic_updater.FrequencyStatusParam(
                                                                            freq_bounds, 0.01, 1))

        rospy.Subscriber('/water_level', Bool, self.sub_water_level)
        freq_bounds = {'min': 4, 'max': 5}  # average rate: 4.919
        self.water_level_freq = diagnostic_updater.HeaderlessTopicDiagnostic("water_level", self.updater,
                                                                             diagnostic_updater.FrequencyStatusParam(
                                                                                 freq_bounds, 0.01, 1))

    def execute(self, ud):
        timer = rospy.Timer(rospy.Duration(0.1), self.diagnosticTimerCallback)
        url = rospy.get_param('/zeta_rest_server', None)
        robot_id = os.getenv('ROBOT_ID')
        try:
            while True:
                start = time.time()
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
                dict2str = json.dumps(self.data_aggr)
                self.pub_data_aggr.publish(dict2str)
                # robot_id = rospy.get_param('/robot_id', "empty") 나중에 param 적용하면 주석제거
                # auth_ones = HTTPBasicAuth('bcc@abc.com', '12341234')
                '''
                if url is not None:
                    try:
                        r = requests.request(
                            method='patch',
                            url=url,
                            data=dict2str,
                            # auth=auth_ones,
                            verify=None,
                            headers={'Content-type': 'application/json'}
                        )
                        if r is not None:
                            if r.status_code != 200:
                                res = r.reason
                                print('patch/' + r.reason)
                            else:
                                print(r.status_code)
                    except Exception as e:
                        print(e)
                else:
                    url = rospy.get_param('/zeta_rest_server', None)
                '''
                data = {'air': self.air, 'battery97': self.battery97, 'battery96': self.battery96, 'imu': self.imu,
                        'robot_id': robot_id}
                if 'air' in self.data_aggr:
                    data['air']['level'] = self.data_aggr['air']['level']
                    data['air']['freq'] = self.data_aggr['air']['freq']
                if 'auto_charge_ino_value' in self.data_aggr:
                    data['auto_charge_ino_value'] = self.data_aggr['auto_charge_ino_value']
                if 'battery97' in self.data_aggr:
                    data['battery97']['level'] = self.data_aggr['battery97']['level']
                    data['battery97']['freq'] = self.data_aggr['battery97']['freq']
                if 'battery96' in self.data_aggr:
                    data['battery96']['level'] = self.data_aggr['battery96']['level']
                    data['battery96']['freq'] = self.data_aggr['battery96']['freq']
                if 'emergency_stop' in self.data_aggr:
                    data['emergency_stop'] = self.data_aggr['emergency_stop']
                if 'estop' in self.data_aggr:
                    data['estop'] = self.data_aggr['estop']
                if 'imu' in self.data_aggr:
                    data['imu']['level'] = self.data_aggr['imu']['level']
                    data['imu']['freq'] = self.data_aggr['imu']['freq']
                if 'sonar' in self.data_aggr:
                    data['sonar'] = self.data_aggr['sonar']
                    data['sonar']['value'] = data['sonar']['value'].replace(' ', '')
                    data['sonar']['value'] = data['sonar']['value'][1:-1]
                if 'bumper' in self.data_aggr:
                    data['bumper'] = self.data_aggr['bumper']
                if 'water_level' in self.data_aggr:
                    data['water_level'] = self.data_aggr['water_level']

                data2str = json.dumps(data)
                future = producer.send(
                    topic="zetabot", value=data2str.encode("utf8")
                )
                try:
                    record_metadata = future.get(timeout=10)
                    print(record_metadata)
                except KafkaError:
                    print("kafka producer error")

                elapseTime = time.time() - start
                print('elapseTime: {}'.format(elapseTime))
                time.sleep(3 - elapseTime)
        except KeyboardInterrupt:
            print("Interrupted by user, shutting down...")

        timer.shutdown()
        return 'succeeded'

    def data_aggr_summary(self, stat):
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "It's OK")
        stat.add("air", self.air)
        stat.add("auto_charge_ino_value", self.auto_charge_ino_value)
        stat.add("battery97", self.battery97)
        stat.add("battery96", self.battery96)
        stat.add("battery_soc", self.battery_soc)
        stat.add("emergency_stop", self.emergency_stop)
        stat.add("estop", self.estop)
        stat.add("imu", self.imu)
        stat.add("sonar", self.sonar)
        stat.add("bumper", self.bumper)
        stat.add("water_level", self.water_level)

    def diagnosticTimerCallback(self, event):
        self.updater.update()

    def sub_air(self, msg):
        self.air = {
            'Dust_PM2_5_ugm3': msg.Dust_PM2_5_ugm3,
            'Dust_PM10_ugm3': msg.Dust_PM10_ugm3,
            'CO2_ppm': msg.CO2_ppm,
            'HCHO_ugm3': msg.HCHO_ugm3,
            'CO_ppm': msg.CO_ppm,
            'NO2_ppm': msg.NO2_ppm,
            'Rn_Bqm3': msg.Rn_Bqm3,
            'TVOCs_ugm3': msg.TVOCs_ugm3,
            'temp_celcius': msg.temp_celcius,
            'hum_RHp': msg.hum_RHp,
            'Ozone_ppm': msg.Ozone_ppm
        }
        self.air_freq.tick()

    def sub_auto_charge_state_ino(self, msg):
        self.auto_charge_ino_value = msg.data
        self.auto_charge_INO_freq.tick()

    def sub_battery(self, msg):
        battery = {
            'id': msg.id,
            'voltage': msg.voltage,
            'current': msg.current,
            'status': msg.status,
            'time_to_full': msg.time_to_full,
            'time_to_empty': msg.time_to_empty,
            'SOC': msg.SOC,
            'SOH': msg.SOH,
            'remaining_capacity': msg.remaining_capacity,
            'available_energy': msg.available_energy,
            'temperature': msg.temperature,
        }
        if msg.id == 97:
            self.battery97 = battery
        if msg.id == 96:
            self.battery96 = battery

        self.battery_freq.tick()

    def sub_battery_soc(self, msg):
        self.battery_freq_soc_freq.tick()
        self.battery_soc = msg.data

    def sub_emergency_stop(self, msg):
        self.emergency_stop_freq.tick()
        self.emergency_stop = msg.data

    def sub_eStop(self, msg):
        self.estop_freq.tick()
        self.estop = msg.data

    def sub_imu(self, msg):
        self.imu = {
            "orientation_x": msg.orientation.x,
            "orientation_y": msg.orientation.y,
            "orientation_z": msg.orientation.z,
            "orientation_w": msg.orientation.w,
            "angular_velocity_x": msg.angular_velocity.x,
            "angular_velocity_y": msg.angular_velocity.y,
            "angular_velocity_z": msg.angular_velocity.z,
            "linear_acceleration_x": msg.linear_acceleration.x,
            "linear_acceleration_y": msg.linear_acceleration.y,
            "linear_acceleration_z": msg.linear_acceleration.z,
        }
        self.imu_freq.tick()

    def sub_scan(self, msg):
        self.scan_freq.tick()

    def sub_sonar(self, msg):
        self.sonar = msg.data
        self.sonar_freq.tick()

    def sub_bumper(self, msg):
        self.bumper = msg.data
        self.bumper_freq.tick()

    def sub_water_level(self, msg):
        self.water_level = msg.data
        self.water_level_freq.tick()

    def _topic_status(self, status, keyword):
        tmp = {'level': status.level}
        for value in status.values:
            if value.key == "Actual frequency (Hz)":
                tmp['freq'] = value.value
                break
        if keyword in self.data:
            tmp['value'] = self.data[keyword]
        self.data_aggr[keyword] = tmp

    def sub_diagnostics(self, msg):
        for status in msg.status:
            if status.name == "zetabot_diagnostics: data aggregation":  # rosbag play시 중복 확인
                for value in status.values:
                    self.data[value.key] = value.value
            elif status.name == 'zetabot_diagnostics: air topic status':
                self._topic_status(status, 'air')
            elif status.name == 'zetabot_diagnostics: auto_charge_INO topic status':
                self._topic_status(status, 'auto_charge_ino_value')
            elif status.name == 'zetabot_diagnostics: battery topic status':
                self._topic_status(status, 'battery96')
                self._topic_status(status, 'battery97')
            elif status.name == 'zetabot_diagnostics: battery_SOC topic status':
                self._topic_status(status, 'battery_soc')
            elif status.name == 'zetabot_diagnostics: emergency_stop topic status':
                self._topic_status(status, 'emergency_stop')
            elif status.name == 'zetabot_diagnostics: estop topic status':
                self._topic_status(status, 'estop')
            elif status.name == 'zetabot_diagnostics: imu topic status':
                self._topic_status(status, 'imu')
            elif status.name == 'zetabot_diagnostics: scan topic status':
                self._topic_status(status, 'scan')
            elif status.name == 'zetabot_diagnostics: sonar topic status':
                self._topic_status(status, 'sonar')
            elif status.name == 'zetabot_diagnostics: bumper topic status':
                self._topic_status(status, 'bumper')
            elif status.name == 'zetabot_diagnostics: water_level topic status':
                self._topic_status(status, 'water_level')
        # print(self.data_aggr)


def main(config):
    smach.set_loggers(log, log, log, log)  # disable
    rospy.init_node('zetabot_diagnostics', log_level=rospy.DEBUG, disable_signals=True)
    top = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'timeout'])
    top.userdata.blackboard = init_blackboard()

    with top:
        smach.StateMachine.add('start', test_diagnostics(), {'succeeded': 'delay'})
        smach.StateMachine.add('delay', preempted_timeout(3), {'succeeded': 'start'})

    outcome = top.execute()
    rospy.signal_shutdown('done!')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="",
                                     usage="",
                                     epilog="",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-d', '--device', type=str, default='/dev/device', help='device path to check')
    parser.add_argument('-n', '--name', type=str, default='node_name', help='node name to lanch')
    parser.add_argument('-p', '--package', type=str, default='package_name', help='package name to lanch')
    parser.add_argument('-e', '--executable', type=str, default='node_executable', help='node executable to lanch')
    parser.add_argument('-a', '--args', type=str, default='', help='arguments of node')
    parser.add_argument('-t', '--timeout', type=int, default=5, help='wait time after nodoe exit')
    parser.add_argument('-o', '--option', type=str, default='', help='for device options')
    parser.add_argument('-c', '--topic', type=str, default='', help='for Subscribe')

    parsed_known_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    main(parsed_known_args)
