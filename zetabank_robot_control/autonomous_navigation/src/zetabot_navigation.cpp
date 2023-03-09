/*
 * zetabot navigation
 *
 *  Created on: 2018. 10. 07.
 *      Author: Geonhee-LEE
 */

#include "autonomous_navigation/zetabot_navigation.h"


ZetabotNavigation::ZetabotNavigation()
{
	cout << "##### start Zetabot Navigation ##### " <<endl;

	statePublisher = nh_.advertise<std_msgs::String>("/zetabot_navigation/state", 1000);
	stopPathPublisher = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
	cmdVelPublisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    //setGoalSubscriber = nh_.subscribe("/zetabot_navigation/set_goal", 1000, &ZetabotNavigation::setGoal, this);
	stopSubscriber = nh_.subscribe("/zetabot_navigation/stop", 1000, &ZetabotNavigation::stop, this);
	globalPlanSubscriber = nh_.subscribe("/move_base/GlobalPlanner/plan", 1000, &ZetabotNavigation::checkGlobalPlan, this);
	currentGoalSubscriber = nh_.subscribe("/move_base/current_goal", 1000, &ZetabotNavigation::setCurrentGoal, this);
	currentPoseSubscriber = nh_.subscribe("/amcl_pose", 1000, &ZetabotNavigation::setCurrentPose, this);

	clearCostMap = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    
	ac = new MoveBaseClient("move_base", true);
	//wait for the action server to come up
	while(!ac->waitForServer(ros::Duration(5.0)))
    {
		ROS_INFO("Waiting for the move_base action server to come up");
	}

    sendgoal_flgs = false;
	cout << "##### zetabot navigation connect finished #####" <<endl;
}


ZetabotNavigation::~ZetabotNavigation()
{

}

void ZetabotNavigation::recoveryBehavior() 
{
    
	ros::Duration(2).sleep();
	ROS_INFO("zetabot recoveryBehavior start");

	// Reading ultrasonic sensor
    /*
    CUltrasonicData uData = ultra->readData(0);
	ROS_INFO("uData.getData()[4] %d",uData.getData()[3]);
	if(uData.getData()[4] >= 70 && uData.getData()[3] >= 70 && uData.getData()[5] >= 70) 
    {
		speak("back");
		ROS_INFO("move backward");
		wheel->moveByVelocityXYT(-150,0,0);
		ros::Duration(1).sleep();
		ROS_INFO("move stop");
		wheel->stop();
	} else if(uData.getData()[2] >= 70 ||uData.getData()[6] >= 70) 
    {
		speak("side");
		ROS_INFO("move side");
		if(uData.getData()[2] > uData.getData()[6]) 
        {
			wheel->moveByVelocityXYT(0,-150,0);
		} 
        else 
        {
			wheel->moveByVelocityXYT(0,150,0);
		}
		ros::Duration(1).sleep();
		ROS_INFO("move stop");
		wheel->stop();
	} 
    else 
    {
		ros::Duration(1).sleep();
	}
    */    
}

bool ZetabotNavigation::setGoal(geometry_msgs::TransformStamped transformStamped) 
{
	std_msgs::String strMsg;
	strMsg.data= "start";
	statePublisher.publish(strMsg);

	move_base_msgs::MoveBaseGoal goal;
	//-1.368, 0.035, 0.000), Orientation(0.000, 0.000, 0.737, 0.676) = Angle: 1.658
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = transformStamped.transform.translation.x;
	goal.target_pose.pose.position.y = transformStamped.transform.translation.y;
	goal.target_pose.pose.position.z = transformStamped.transform.translation.z;
    goal.target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
    goal.target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
    goal.target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
    goal.target_pose.pose.orientation.w = transformStamped.transform.rotation.w;

	actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::ABORTED);
	int failCount = 0;

	state = actionlib::SimpleClientGoalState::ABORTED;
	isStop = false;

	while(state != actionlib::SimpleClientGoalState::SUCCEEDED && !isStop) 
    {        
        tfb.sendTransform(transformStamped);

		ROS_INFO("Sending goal");
		ac->sendGoal(goal);
        sendgoal_flgs = true;

		ros::Duration(2).sleep();
		if(isStop) 
        {
			break;
		}
		ac->waitForResult();

		ros::Duration(1).sleep();

		state = ac->getState();

		if(isStop)         
			break;
		
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
			ROS_INFO("########## Move goal success ##########");
            return true;
        }
		else
			ROS_ERROR("########## Fail to move goal ##########");

		if(state != actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
			//clearCostMap.call(em);
							failCount++;
			//				if(failCount >1) {
			recoveryBehavior();
			clearCostMap.call(em);
			//				}
            if(failCount > 3)
                return false;
		}
		sendgoal_flgs = false;
	}

	strMsg.data= "finish";
	statePublisher.publish(strMsg);
}

void ZetabotNavigation::stop(actionlib_msgs::GoalID goal) 
{
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0;
	cmd_vel.angular.x = 0;
	cmdVelPublisher.publish(cmd_vel);
	stopPathPublisher.publish(goal);

	isStop = true;
    sendgoal_flgs = false;

	cout << "########## Stop Called ########## " << endl;
}

void ZetabotNavigation::teleop(geometry_msgs::Twist cmd_vel)
{
	cmdVelPublisher.publish(cmd_vel);
}

void ZetabotNavigation::checkGlobalPlan(nav_msgs::Path path) 
{
//	cout << "path.header.seq : " << path.header.seq << endl;
//	cout << "path.poses.size() " << path.poses.size() << endl;

	int pathSize = path.poses.size();
	actionlib_msgs::GoalID stopGoal;
    /*
    if(oldPathPoseSize > 200) 
    {
		if(pathSize > oldPathPoseSize*2) 
        {
            cout << "pose count 2X over STOP!!!" << endl;
            stopPublisher.publish(stopGoal);
             oldPathPoseSize = 9999;
            return;
		}
	} 
    else if(oldPathPoseSize < 50) 
    {		
        if(pathSize > 100) 
        {
            cout << "pose count small to over STOP!!!" << endl;
            stopPublisher.publish(stopGoal);
            oldPathPoseSize = 9999;
            return;
	    }
    }*/

//	 cout << currentGoal.pose.position.x <<" , " << currentPose.pose.pose.position.x<<" , " <<  currentGoal.pose.position.y<<" , " <<  currentPose.pose.pose.position.y << endl;
	float distance = calculateDistance(currentGoal.pose.position.x, currentGoal.pose.position.y, currentPose.pose.pose.position.x, currentPose.pose.pose.position.y);

	cout << "Total Distance: " << distance << ", pathSize : " << pathSize << endl;
	/*
    for(int i=0; i<pathSize; i++) 
    {
		float tmpPosX = path.poses[i].pose.position.x;
		float tmpPosY = path.poses[i].pose.position.y;
	}*/

	if(distance >= 10) 
    {
		if(pathSize >= distance*70) 
        {
			//stopPublisher.publish(stopGoal);
			//speak("over");
		}
	} 
    else if(distance >= 0.35) 
    {
		if(pathSize >= distance*90) 
        {
			//stopPublisher.publish(stopGoal);
			//speak("over");
		}
	}

	oldPathPoseSize = pathSize;
}

float ZetabotNavigation::calculateDistance(float x1, float y1, float x2, float y2) 
{
	float distance = sqrtf(pow(x1-x2, 2.0)+ pow(y1-y2,2.0));

	return distance;
}

void ZetabotNavigation::setCurrentGoal(geometry_msgs::PoseStamped goal) 
{
	oldPathPoseSize = 9999;
	currentGoal = goal;
}

void ZetabotNavigation::setCurrentPose(geometry_msgs::PoseWithCovarianceStamped pose) {
	currentPose = pose;
}
