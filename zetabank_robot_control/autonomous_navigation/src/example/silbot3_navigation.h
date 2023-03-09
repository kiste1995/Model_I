/*
 * silbot3_navigation.h
 *
 *  Created on: 2016. 2. 1.
 *      Author: zikprid
 */


#ifndef __SILBOT3_NAVIGATION_H__
#define __SILBOT3_NAVIGATION_H__

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <robocare/device/client/CWheelClient.h>
#include <robocare/device/client/CUltraClient.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

namespace silbot3 {

	class Silbot3Navigation {


	private:
		CWheel* wheel;
		CUltrasonic* ultra;
		ros::ServiceClient clearCostMap;
		ros::Publisher mentPublisher;
		ros::Publisher statePublisher;
		ros::Subscriber setGoalSubscriber;
		ros::Subscriber stopSubscriber;
		ros::Subscriber globalPlanSubscriber;
		ros::Subscriber currentGoalSubscriber;
		ros::Subscriber currentPoseSubscriber;
		ros::Publisher stopPublisher;

		std_srvs::Empty em;
		MoveBaseClient* ac;

		geometry_msgs::PoseStamped currentGoal;
		geometry_msgs::PoseWithCovarianceStamped currentPose;

		bool isStop;
		int oldPathPoseSize;

	public:
		Silbot3Navigation();
		virtual ~Silbot3Navigation();

		void speak(string sentence);
		void recoveryBehavior();
		void setGoal(move_base_msgs::MoveBaseGoal org);
		void stop(actionlib_msgs::GoalID goal);
		void checkGlobalPlan(nav_msgs::Path path);
		void sendState();
		void setCurrentGoal(geometry_msgs::PoseStamped goal);
		void setCurrentPose(geometry_msgs::PoseWithCovarianceStamped goal);

		float calculateDistance(float x1, float y1, float x2, float y2);
	};

} /* namespace behavior */

#endif /* __SILBOT3_NAVIGATION_H__ */

