import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

from deco import ros

from zetabank_msgs.msg import BreakTurn

@ros
class AutoChargePublisher:

    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.autocharge_publisher = rospy.Publisher('/autocharge_state_NUC', UInt8, queue_size = 1)
        self.initial_position_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.led_control_publisher = rospy.Publisher('/robot_mode', String, queue_size = 1)

        self.breakturn_publisher = rospy.Publisher('/break_turn', BreakTurn, queue_size = 1)


    def breakturn_pub(self, state):
        btmsg = BreakTurn()
        btmsg.breakturn = state
        self.breakturn_publisher.publish(btmsg)


    def autocharge_pub(self, step_num):
        pub_rate = rospy.Rate(25) #5hz

        #rospy.loginfo(step_num)
        self.autocharge_publisher.publish(step_num)
        pub_rate.sleep()

    def velocity_pub(self, flag, x, z):
        pub_rate = rospy.Rate(25) #5hz

        if flag:
            movemsg = Twist()
            movemsg.linear.x = x
            movemsg.angular.z = z

            self.velocity_publisher.publish(movemsg)
        else:
            pass

        pub_rate.sleep()

    def led_control_pub(self, mode):
        pub_rate = rospy.Rate(25)

        rospy.loginfo(mode)
        self.led_control_publisher.publish(mode)
        pub_rate.sleep()

    def initial_position_pub(self):
        start_pos = PoseWithCovarianceStamped()
        start_pos.header.frame_id = "map"
        start_pos.header.stamp = rospy.Time.now()

        start_pos.pose.pose.position.x = self.initial_position_x
        start_pos.pose.pose.position.y = self.initial_position_y
        start_pos.pose.pose.position.z = 1.0

        start_pos.pose.pose.orientation.x = 0.0
        start_pos.pose.pose.orientation.y = 0.0
        start_pos.pose.pose.orientation.z = self.initial_orientation_z
        start_pos.pose.pose.orientation.w = self.initial_orientation_w

        start_pos.pose.covariance[0] = 0.25
        start_pos.pose.covariance[7] = 0.25
        start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[35] = 0.06853891945200942

        rospy.loginfo(start_pos)
        rospy.sleep(1)
        self.initial_position_publisher.publish(start_pos)
        rospy.sleep(1)

        return True