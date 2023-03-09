/*
 * silbot3_navigation.cpp
 *
 *  Created on: 2015. 11. 26.
 *      Author: robocare
 */

#include "silbot3_navigation.h"
#include <math.h>

using namespace silbot3;

Silbot3Navigation::Silbot3Navigation() {
	ros::NodeHandle nodeHandle;
	mentPublisher = nodeHandle.advertise<std_msgs::String>("/silbot3_navigation/ment", 1000);
	statePublisher = nodeHandle.advertise<std_msgs::String>("/silbot3_navigation/state", 1000);
	stopPublisher = nodeHandle.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);

	setGoalSubscriber = nodeHandle.subscribe("/silbot3_navigation/set_goal", 1000, &Silbot3Navigation::setGoal, this);
	stopSubscriber = nodeHandle.subscribe("/silbot3_navigation/stop", 1000, &Silbot3Navigation::stop, this);
	globalPlanSubscriber = nodeHandle.subscribe("/move_base/NavfnROS/plan", 1000, &Silbot3Navigation::checkGlobalPlan, this);
	currentGoalSubscriber = nodeHandle.subscribe("/move_base/current_goal", 1000, &Silbot3Navigation::setCurrentGoal, this);
	currentPoseSubscriber = nodeHandle.subscribe("/amcl_pose", 1000, &Silbot3Navigation::setCurrentPose, this);


	oldPathPoseSize = 9999;

	wheel= CWheelClientProxy::newInstance();
	ultra	= CUltrasonicClientProxy::newInstance();

	ac = new MoveBaseClient("move_base", true);
	//wait for the action server to come up
	while(!ac->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	clearCostMap = nodeHandle.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");



	cout << "silbot3 navigation connect finished" <<endl;


}

Silbot3Navigation::~Silbot3Navigation() {

}

void Silbot3Navigation::speak(string sentence) {
	std_msgs::String strmsg;
	strmsg.data = sentence;

	mentPublisher.publish(strmsg);

	cout <<sentence <<endl;
}

void Silbot3Navigation::recoveryBehavior() {
	speak("fail");
	ros::Duration(2).sleep();
	ROS_INFO("robocare recoveryBehavior start");
	CUltrasonicData uData = ultra->readData(0);
	ROS_INFO("uData.getData()[4] %d",uData.getData()[3]);
	if(uData.getData()[4] >= 70 && uData.getData()[3] >= 70 && uData.getData()[5] >= 70) {
		speak("back");
		ROS_INFO("move backward");
		wheel->moveByVelocityXYT(-150,0,0);
		ros::Duration(1).sleep();
		ROS_INFO("move stop");
		wheel->stop();
	} else if(uData.getData()[2] >= 70 ||uData.getData()[6] >= 70) {
		speak("side");
		ROS_INFO("move side");
		if(uData.getData()[2] > uData.getData()[6]) {
			wheel->moveByVelocityXYT(0,-150,0);
		} else {
			wheel->moveByVelocityXYT(0,150,0);
		}
		ros::Duration(1).sleep();
		ROS_INFO("move stop");
		wheel->stop();
	} else {
		ros::Duration(1).sleep();
	}
	speak("restart");

}

void Silbot3Navigation::setGoal(move_base_msgs::MoveBaseGoal org) {
	cout << org.target_pose.pose.position.x <<"," << org.target_pose.pose.position.y<<"," << org.target_pose.pose.position.z<<endl;
	cout << org.target_pose.pose.orientation.x<<","  << org.target_pose.pose.orientation.y<<","  << org.target_pose.pose.orientation.z<<"," << org.target_pose.pose.orientation.w<<endl;

	std_msgs::String strMsg;
	strMsg.data= "start";
	statePublisher.publish(strMsg);

	move_base_msgs::MoveBaseGoal goal;
	//-1.368, 0.035, 0.000), Orientation(0.000, 0.000, 0.737, 0.676) = Angle: 1.658
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = org.target_pose.pose.position.x;
	goal.target_pose.pose.position.y = org.target_pose.pose.position.y;
	goal.target_pose.pose.position.z = org.target_pose.pose.position.z;
	goal.target_pose.pose.orientation.x = org.target_pose.pose.orientation.x;
	goal.target_pose.pose.orientation.y = org.target_pose.pose.orientation.y;
	goal.target_pose.pose.orientation.z = org.target_pose.pose.orientation.z;
	goal.target_pose.pose.orientation.w = org.target_pose.pose.orientation.w;


	actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::ABORTED);
	int failCount = 0;

	state = actionlib::SimpleClientGoalState::ABORTED;
	failCount = 0;
	isStop = false;
	while(state != actionlib::SimpleClientGoalState::SUCCEEDED && !isStop) {
		ROS_INFO("Sending goal");
		ac->sendGoal(goal);

		ros::Duration(2).sleep();

		if(isStop) {
			break;
		}
		ac->waitForResult();

		ros::Duration(1).sleep();

		state = ac->getState();
		if(isStop) {
			break;
		}
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("move goal1 success");
		else
			ROS_INFO("fail to move goal");

		if(state != actionlib::SimpleClientGoalState::SUCCEEDED) {
			//clearCostMap.call(em);
			//				failCount++;
			//				if(failCount >1) {
			recoveryBehavior();
			clearCostMap.call(em);
			//				}
		}
	}

	strMsg.data= "finish";
	statePublisher.publish(strMsg);
}

void Silbot3Navigation::stop(actionlib_msgs::GoalID goal) {

	cout << "Stop Called! " << endl;
	isStop = true;

	ros::Duration(1).sleep();

	stopPublisher.publish(goal);
}

void Silbot3Navigation::checkGlobalPlan(nav_msgs::Path path) {
//	cout << "path.header.seq : " << path.header.seq << endl;
//	cout << "path.poses.size() " << path.poses.size() << endl;

	int pathSize = path.poses.size();
	actionlib_msgs::GoalID stopGoal;

//	if(oldPathPoseSize > 200) {
//		if(pathSize > oldPathPoseSize*2) {
//			cout << "pose count 2X over STOP!!!" << endl;
//			stopPublisher.publish(stopGoal);
//			oldPathPoseSize = 9999;
//			return;
//		}
//	} else if(oldPathPoseSize < 50) {
//		if(pathSize > 100) {
//			cout << "pose count small to over STOP!!!" << endl;
//			stopPublisher.publish(stopGoal);
//			oldPathPoseSize = 9999;
//			return;
//		}
//	}

//	 cout << currentGoal.pose.position.x <<" , " << currentPose.pose.pose.position.x<<" , " <<  currentGoal.pose.position.y<<" , " <<  currentPose.pose.pose.position.y << endl;
	float distance = calculateDistance(currentGoal.pose.position.x, currentGoal.pose.position.y, currentPose.pose.pose.position.x, currentPose.pose.pose.position.y);

	cout << "distance : " << distance << ", pathSize : " << pathSize << endl;
//	for(int i=0; i<pathSize; i++) {
//		float tmpPosX = path.poses[i].pose.position.x;
//		float tmpPosY = path.poses[i].pose.position.y;
//
//
//	}
	if(distance >= 10) {
		if(pathSize >= distance*70) {
			//stopPublisher.publish(stopGoal);
			speak("over");
		}
	} else if(distance >= 0.35) {
		if(pathSize >= distance*90) {
			//stopPublisher.publish(stopGoal);
			speak("over");
		}
	}



	oldPathPoseSize = pathSize;

}

float Silbot3Navigation::calculateDistance(float x1, float y1, float x2, float y2) {
	float distance = sqrtf(pow(x1-x2, 2.0)+ pow(y1-y2,2.0));

	return distance;
}


void Silbot3Navigation::setCurrentGoal(geometry_msgs::PoseStamped goal) {
	oldPathPoseSize = 9999;
	currentGoal = goal;
}

void Silbot3Navigation::setCurrentPose(geometry_msgs::PoseWithCovarianceStamped pose) {
	currentPose = pose;
}


int main(int argc, char** argv) {
	cout << "start silbot3 navigation" <<endl;

	ros::init(argc, argv, "silbot3_navigation");
	//tell the action client that we want to spin a thread by default
	Silbot3Navigation s3n;

//	0:1.3494|-0.2896|0|0|0|0.3811|0.9245
//	1:3.2497|0.9589|0|0|0|-0.992|0.126

	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	spinner.spin(); // spin() will not return until the node has been shutdown

	return 0;
}
