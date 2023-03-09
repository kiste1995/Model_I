#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cstdio>
#include <queue>
#include <iostream>
#include <stdlib.h>
#include <ctime>

#define POINTNUM            50
#define PI                  3.1415926535897932384626433832795
#define MATH_RAD2DEG        57.2957795f
#define MATH_DEG2RAD        0.0174532f


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class naviWayPoint
{
    struct goal
    {
        float x;
        float y;
        float z;
        float ang;
    };

    public:
        naviWayPoint(ros::NodeHandle nh);
        ~naviWayPoint();
        void sendGoals(void);

        //MoveBaseClient *ac;

        bool b_wpfile_readOK;


    private:
        std::queue<goal> waypoints_queue;
        std::string waypoints_fname;
        ros::NodeHandle _nh;        
        int wp_count;
        bool b_has_waypoint_fname;

        void readWaypoint(void);

};

naviWayPoint::naviWayPoint(ros::NodeHandle nh) : _nh(nh)
{
    if(nh.getParam("waypoints_filename", waypoints_fname))
    {
        b_has_waypoint_fname = true;
        b_wpfile_readOK = false;
        wp_count = 0;

        readWaypoint();

    }
    else 
    {
        b_has_waypoint_fname = false;
        ROS_WARN("No wayoints file name(%s) given as parameter.", waypoints_fname.c_str());        
    }
}

naviWayPoint::~naviWayPoint()
{

}

void naviWayPoint::readWaypoint()
{
    waypoints_queue = std::queue<goal> ();

    struct goal wp;

    char fname[100] = {0};
    char buff[50] = {0};

    sprintf(fname, "%s", waypoints_fname.c_str());
    ROS_INFO("file name : %s", fname);

    FILE *fp = fopen(fname, "r");

    if(fp != NULL)
    {
        while(!feof(fp))
        {
            memset(buff, 0, 50);
            fread(buff, sizeof(char), 50, fp);

            sscanf(buff, "%f,%f,%f,%f", &wp.x, &wp.y, &wp.z, &wp.ang);

            waypoints_queue.push(wp);

            wp_count++;

            ROS_INFO("read wp : (%f, %f, %f, %f)", wp.x, wp.y, wp.z, wp.ang);

        }

        fclose(fp);

        ROS_INFO("Finished reading file %s : count(%d)", waypoints_fname.c_str(), wp_count);

        b_wpfile_readOK = true;
    }
    else 
    {
        ROS_ERROR_STREAM("File open error.");
        b_wpfile_readOK = false;
    }
}

void naviWayPoint::sendGoals(void)
{
    unsigned int stime;
    move_base_msgs::MoveBaseGoal goal;

    MoveBaseClient ac("move_base", true);

    //ac = new MoveBaseClient("move_base_node", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");        
    }


    while(!waypoints_queue.empty())
    {
        goal.target_pose.header.frame_id = "map";
        //goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = waypoints_queue.front().x;
        goal.target_pose.pose.position.y = waypoints_queue.front().y;
        goal.target_pose.pose.position.z = waypoints_queue.front().z;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoints_queue.front().ang);

        ROS_INFO("Sending goal : (%lf, %lf, %lf, %lf)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
            goal.target_pose.pose.position.z, waypoints_queue.front().ang);

        ac.sendGoal(goal);
        stime = clock();

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Reached point...");
            ROS_INFO("Time taken:%lu", clock()-stime);

            waypoints_queue.pop();
        }
        else
        {
            ROS_INFO("Failed to complete waypoint goal.");
        }        

    }

    ROS_INFO("Waypoint goal complete...");

}

int main(int argc, char** argv)
{
    int rate_wc = 100;

    ros::init(argc, argv, "Navigation way point node");

    ROS_INFO("Run navigation way point.");

    ros::NodeHandle node_obj("~");

    naviWayPoint naviWP(node_obj);

    if(naviWP.b_wpfile_readOK == true)
    {
        naviWP.sendGoals();
        ros::spin();
    }
    else
    {
        ROS_INFO("Exit makeing way point process !!!");    
    }
      

    //ROS_INFO("Exit makeing way point process !!!");

    /*if(naviWP.b_has_waypoint_fname == true)
    {
        naviWP.ac->cancelGoal();
    }*/

    return 0;
}
