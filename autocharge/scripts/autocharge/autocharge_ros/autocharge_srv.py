import rospy
from zetabank_msgs.srv import TurnSrv
from zetabot_main.srv import InitPoseSrv

from deco import ros

@ros
class AutoChargeSurvice:
    def __init__(self):
        self.initial_position_survice = rospy.ServiceProxy('/init_pose_srv', InitPoseSrv)
        self.autocharge_start_turn = rospy.ServiceProxy('/turn', TurnSrv)