/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Russell Toris - rctoris@wpi.edu
 * \date April 3, 2014
 */

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <zetabank_msgs/TurnSrv.h>
#include "zetabank_msgs/TurnQuaternionSrv.h"
#include <boost/thread/thread.hpp>

 #include <mutex>
 std::mutex hangle_mutex;

/*!
 * Creates and runs the robot_pose_publisher node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

#define PI_SQ               (M_PI*2.0)

#define MATH_DEG2RAD        0.0174532f
#define MATH_RAD2DEG        57.2957795f

int headingAngle = 0;
// double prev_headingAngle = 0.0;
double Kp = 0.4;
bool b_RunTurnung = false;

ros::Publisher cmdvel_pub;

int modAngle(int ang)
{
    int ret_ang;
    if(ang < 0)
    {
        ret_ang = (360 % (abs(ang)))* -1;
    }
    else {
        ret_ang = 360 % ang;
    }

    return ret_ang;
}

void RunTurning(int *ang)
{
    int goal_hangle = *ang;
    int diff_hangle = 0;
    int hAngle = 0;
    double target_angspeed;

    ros::Time::init();

    ros::Rate loop_rate(50);

    std::lock_guard<std::mutex> guard(hangle_mutex);
    hAngle = headingAngle;

    diff_hangle = modAngle(goal_hangle - hAngle);
    // diff_hangle = 360 % (goal_hangle - hAngle);
    target_angspeed = Kp *(diff_hangle * MATH_DEG2RAD );

    ROS_INFO("[1] hangle:%d dangle:%d tangspeed:%.2lf", hAngle, diff_hangle, target_angspeed);

    geometry_msgs::Twist twist;

    while( abs(diff_hangle) >= 2)
    {
      twist.angular.z = target_angspeed;
      cmdvel_pub.publish(twist);

      std::lock_guard<std::mutex> guard(hangle_mutex);
      hAngle = headingAngle;

      diff_hangle = modAngle(goal_hangle - hAngle);
    //   diff_hangle = 360 % (goal_hangle - hAngle);
      target_angspeed = Kp *(diff_hangle * MATH_DEG2RAD);

      ROS_INFO("[2] hangle:%d dangle:%d tangspeed:%.2lf", hAngle, diff_hangle, target_angspeed);

      loop_rate.sleep();

    }

    b_RunTurnung = false;
}

bool TrunService(zetabank_msgs::TurnSrv::Request & req, zetabank_msgs::TurnSrv::Response &res)
{
  ros::Rate loop_rate(10);

  ROS_INFO("Receive turn service...");

  if(b_RunTurnung == false)
  {
    b_RunTurnung = true;

    int goal_hangle = req.degree;
    int diff_hangle = 0;
    int hAngle = 0;
    double target_angspeed;

    ROS_INFO("Turn angle : %d", goal_hangle);   

    // prev_headingAngle = headingAngle;
    hAngle = headingAngle;

    boost::thread thread_b(RunTurning, &hAngle);

    while(1)
    {
        if(b_RunTurnung == false)
            break;

        loop_rate.sleep();
    }

    /*diff_hangle = modAngle(goal_hangle - hAngle);
    // diff_hangle = 360 % (goal_hangle - hAngle);
    target_angspeed = Kp *(diff_hangle * MATH_DEG2RAD );

    ROS_INFO("[1] hangle:%d dangle:%d tangspeed:%.2lf", hAngle, diff_hangle, target_angspeed);

    geometry_msgs::Twist twist;

    while( abs(diff_hangle) >= 2)
    {
      twist.angular.z = target_angspeed;
      cmdvel_pub.publish(twist);

      std::lock_guard<std::mutex> guard(hangle_mutex);
      hAngle = headingAngle;

      diff_hangle = modAngle(goal_hangle - hAngle);
    //   diff_hangle = 360 % (goal_hangle - hAngle);
      target_angspeed = Kp *(diff_hangle * MATH_DEG2RAD);

      ROS_INFO("[2] hangle:%d dangle:%d tangspeed:%.2lf", hAngle, diff_hangle, target_angspeed);

      loop_rate.sleep();
      //ros::spinOnce();
    }*/

    res.result = true;

    // b_RunTurnung = false;

    ROS_INFO("Complete turning process!");

    return true;
  }
}

void headingAngleCallback(const std_msgs::Int16& msg)
{
  std::lock_guard<std::mutex> guard(hangle_mutex);

  headingAngle = msg.data;

  ROS_INFO("[3] hangle:%d", headingAngle);
}

int main(int argc, char ** argv)
{
  // initialize ROS and the node
   ros::Rate rate(10);

  ros::init(argc, argv, "robot_heading_turn");
  
  ros::NodeHandle nh;
  // configuring parameters

  ros::Time::init();

  ros::Subscriber hang_subsciber = nh.subscribe("heading_angle", 10, headingAngleCallback);

  cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::ServiceServer turn_srv = nh.advertiseService("turn", TrunService);

  ROS_INFO("Ready turn service...");

  //ros::spin();

  

  while (nh.ok())
  {
    rate.sleep();
    ros::spinOnce();   

    ;

  }

    ROS_INFO("Exit robot heading turn process !!!");
  return EXIT_SUCCESS;
}
