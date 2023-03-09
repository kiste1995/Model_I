/*
 * zetabot navigation
 *
 *  Created on: 2018. 10. 07.
 *      Author: Geonhee-LEE
 */

#ifndef ZETABANK_NAVIGATION_H_
#define ZETABANK_NAVIGATION_H_

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;


class ZetabotNavigation{
    public:
        ZetabotNavigation(void);
        ~ZetabotNavigation(void);
    private:
        ros::NodeHandle nh_;

		ros::Publisher statePublisher;
		ros::Publisher stopPathPublisher;
		ros::Publisher cmdVelPublisher;

		ros::Subscriber setGoalSubscriber;
		ros::Subscriber stopSubscriber;
		ros::Subscriber globalPlanSubscriber;
		ros::Subscriber currentGoalSubscriber;
		ros::Subscriber currentPoseSubscriber;

		ros::ServiceClient clearCostMap;
        std_srvs::Empty em;
		
        MoveBaseClient* ac;

		geometry_msgs::PoseStamped currentGoal;
		geometry_msgs::PoseWithCovarianceStamped currentPose;
	

		int oldPathPoseSize;
		bool isStop; // represent status of robot

        tf2_ros::TransformBroadcaster tfb;

        actionlib_msgs::GoalID stop_msg;


    public:
        bool sendgoal_flgs; // if sendgoal func execute, it is true
		void teleop(geometry_msgs::Twist);
		void recoveryBehavior();
		bool setGoal(geometry_msgs::TransformStamped org);
		void stop(actionlib_msgs::GoalID goal);
		void checkGlobalPlan(nav_msgs::Path path);
		float calculateDistance(float x1, float y1, float x2, float y2);
		void setCurrentGoal(geometry_msgs::PoseStamped goal);
		void setCurrentPose(geometry_msgs::PoseWithCovarianceStamped goal);
};

#endif // MOVEMENT_STATUS_H_
