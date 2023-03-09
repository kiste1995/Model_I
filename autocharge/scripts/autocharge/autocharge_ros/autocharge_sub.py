import rospy
import math

from std_msgs.msg import Bool, String, Float32, UInt8
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
# from zetabot_main.msg import SonarArray

#ros simple goal lib
import actionlib
from move_base_msgs.msg import MoveBaseActionResult

from deco import ros



@ros
class AutoChargeSubscriber:
    
    def __init__(self):
        self.cancel_flag = False
        self.battery_amount = 0
        self.degree = 0
        self.charging_station_state = ""
        self.goal_status = None
        self.sona_distance = 0
        self.bEStop = False

        self.battery_amount_subscriber = rospy.Subscriber('/battery_SOC', Float32, self._battery_amount_subscriber_callback)
        self.charging_station_subscriber = rospy.Subscriber('/autocharge_state_INO', String, self._charging_station_subscriber_callback)
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self._imu_subscriber_callback)
        self.autocharge_cancel_subscriber = rospy.Subscriber("/charging_cancel", Bool, self._cancel_subscriber_callback)
        self.goal_result_subscriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self._movebase_result_subscriber_callback)
        self.sona_subscriber = rospy.Subscriber('/sonar', Float32MultiArray, self._sona_subscriber_callback)
        
        self.estop_sub = rospy.Subscriber("/EmergencyStop", UInt8, self.CallEStop)

    def CallEStop(self, msg):
        self.bEStop_Status = msg.data
        
        if self.bEStop_Status != 1 and self.bEStop == False:
            if self.bEStop_Status == 0:
                print("Detected Lidar Field\r")
            elif self.bEStop_Status == 3:
                print("Pushed E-Stop Button\r")
            elif self.bEStop_Status == 2:
                print("Detected Lidar Field and Pushed E-Stop Button\r")

            self.bprevEStop = self.bEStop
            self.bEStop = True
            
        elif self.bEStop == True and self.bEStop_Status == 1:
            # elif self.bEStop == True and self.bEStop_Status == 0:
            print("Release E-stop/ Lidar detecting\r")
            
            self.bprevEStop = self.bEStop
            self.bEStop = False
            
    def _battery_amount_subscriber_callback(self, msg):
        self.battery_amount = int(msg.data)
        # 0 ~ 100

    def _charging_station_subscriber_callback(self, msg):
        self.charging_station_state = msg.data

        # start
        # contact
        # not_connected
        # left
        # right

    def _imu_subscriber_callback(self, msg):
        X, Y, Z = self._quaternion_to_euler_angle(msg.orientation)

        #degree range 0 ~ 359
        if Z > 0: 
            self.degree = int(Z) + 180
        else: 
            self.degree = int(Z) + 179

        '''
        print ("Quaternoion ================")
        print ("x : ", msg.orientation.x)
        print ("y : ", msg.orientation.y)
        print ("z : ", msg.orientation.z)
        print ("w : ", msg.orientation.w)
        print ("Euler -----------------------")
        print ("X : ", X)
        print ("Y : ", Y)
        print ("Z : ", Z)
        print("degree : ", self.degree)
        '''

    def _quaternion_to_euler_angle(self, msg):
        # ## for robotheading angle
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

    def _cancel_subscriber_callback(self, data):
        self.cancel_flag = data.data
        print("receive auto charing cancel msg => {}".format(data))    

    def _sona_subscriber_callback(self, data):
        self.sona_distance = int((data.data[2]+data.data[3])/2)
        #print(type(self.sona_distance))
        # print("sonar dist : " + str(self.sona_distance))

        """
        self.sona_left = data[5]
        self.sona_right = data[6]
        print("sona_left: ", self.sona_left)
        print("sona_right: ", self.sona_right)
        """

    def _movebase_result_subscriber_callback(self, data):
        self.goal_status = data.status.status