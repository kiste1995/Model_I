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
#include <zetabank_msgs/BreakTurn.h>
// #include "zetabank_msgs/TurnQuaternionSrv.h"
#include <boost/thread/thread.hpp>

#include <boost/thread/mutex.hpp>
boost::mutex hangle_mutex;

//  #include <mutex>
//  std::mutex hangle_mutex;

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

#define DONE    1
#define BREAK   2

int headingAngle = 0;
double Kp = 0.1;
double Kd = 0.2;
bool b_RunTurnung = false;
int goal_hangle = 0;
double min_angular = 0.1;
double max_angular = 1.2;
std::string state_str;
bool bBreakTurn = false;

ros::Publisher cmdvel_pub;

int modAngle(int ang)
{
    int ret_ang;
    if(ang < 0)
    {
        ret_ang = (abs(ang) % 360)* -1;
        // ROS_INFO("[modAngle - 1] ret_ang:%d", ret_ang);
    }
    else {
        ret_ang = ang % 360;
        // ROS_INFO("[modAngle - 2] ret_ang:%d", ret_ang);
    }

    return ret_ang;
}

void RunTurning(int *rate)
{
  int diff_hangle = 0;
  int hAngle = 0;
  double target_angular;
  int prev_diffhang = 0;
  int sign = 1;

  ros::Time::init();

  ros::Rate loop_rate(*rate);

  geometry_msgs::Twist twist;

  while(ros::ok())
  {
    if(b_RunTurnung == true)
    {
      ROS_INFO("start turning....");

      // std::lock_guard<std::mutex> guard(hangle_mutex);
      hangle_mutex.lock();
      hAngle = headingAngle;
      hangle_mutex.unlock();

      hAngle = modAngle(hAngle);
      diff_hangle = goal_hangle - hAngle;

      if(goal_hangle < 0)
      {
        goal_hangle = 360 + goal_hangle;
        ROS_INFO("[0] goal angle:%d", goal_hangle);
      }
      
      // diff_hangle = modAngle(goal_hangle - hAngle);
      // target_angular = Kp *(diff_hangle * MATH_DEG2RAD);

      ROS_INFO("[1] hangle:%d dangle:%d", hAngle, diff_hangle);
      // ROS_INFO("[1] hangle:%d dangle:%d tangspeed:%.2lf", hAngle, diff_hangle, target_angular);

      while( abs(diff_hangle) > 1)
      { 
        if(bBreakTurn == true)
        {
          ROS_INFO("Break turning....");
          bBreakTurn = false;
          break;
        }
        
        loop_rate.sleep();
        ros::spinOnce();

        // std::lock_guard<std::mutex> guard(hangle_mutex);
        hangle_mutex.lock();
        hAngle = headingAngle;
        hangle_mutex.unlock();

        hAngle = modAngle(hAngle);
        diff_hangle = goal_hangle - hAngle;
        if(diff_hangle < 0)
          sign = -1;
        else 
          sign = 1;
        // diff_hangle = modAngle(goal_hangle - hAngle);
        target_angular = Kp * (diff_hangle * MATH_DEG2RAD) + Kd * ((prev_diffhang - diff_hangle) * MATH_DEG2RAD);

        
        if(fabs(target_angular) > max_angular)
          target_angular = max_angular*sign;      
        if(fabs(target_angular) < min_angular)
          target_angular = min_angular*sign;

        // target_angular = Kp *(diff_hangle * MATH_DEG2RAD);

        ROS_INFO("[2] hangle:%d dangle:%d tangular:%.2lf", hAngle, diff_hangle, target_angular);

        twist.angular.z = target_angular;
        cmdvel_pub.publish(twist);

        prev_diffhang = diff_hangle;

      }

      twist.angular.z = 0.0;
      cmdvel_pub.publish(twist);
      
      b_RunTurnung = false;

      ROS_INFO("b_RunTurnung is false...");

    }
    loop_rate.sleep();
  }
    
}

bool TurnService(zetabank_msgs::TurnSrv::Request & req, zetabank_msgs::TurnSrv::Response &res)
{
  ros::Rate loop_rate(10);

  ROS_INFO("Receive turn service...");

  if(b_RunTurnung == false)
  {
    b_RunTurnung = true;
    ROS_INFO("b_RunTurnung is true...");

    goal_hangle = req.degree;
    ROS_INFO("Turn angle : %d", goal_hangle);   

    while(1)
    {
        if(b_RunTurnung == false)
        {
          if(bBreakTurn == true)
            res.result = BREAK;
          else
            res.result = DONE;

          ROS_INFO("Complete turning process!");

          break;
        }

        loop_rate.sleep();
    }

    return true;
  }
}

void headingAngleCallback(const std_msgs::Int16& msg)
{
  // std::lock_guard<std::mutex> guard(hangle_mutex);
  hangle_mutex.lock();
  headingAngle = msg.data;
  hangle_mutex.unlock();

    // ROS_INFO("[3] hangle:%d", headingAngle);

}

void breakTurnCallback(const zetabank_msgs::BreakTurn& msg)
{
  bBreakTurn = msg.breakturn;
}

int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot_heading_turn");
  
  ros::NodeHandle nh;
  // configuring parameters

  ros::Time::init();

  ros::Rate rate(10);

  ros::Subscriber hang_subsciber = nh.subscribe("heading_angle", 10, headingAngleCallback);

  ros::Subscriber breakturn_subsciber = nh.subscribe("break_turn", 10, breakTurnCallback);

  cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::ServiceServer turn_srv = nh.advertiseService("turn", TurnService);

  ROS_INFO("Ready turn service...");

  int run_rate = 50;

  boost::thread thread_b(RunTurning, &run_rate);

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
