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

#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <zetabank_msgs/TurnSrv.h>
#include "zetabank_msgs/TurnQuaternionSrv.h"

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

double headingAngle = 0.0;
// double prev_headingAngle = 0.0;
double Kp = 0.3;
double Kd = 0.3;
bool b_RunTurnung = false;
double min_angular = 0.5;
double max_angular = 1.5;
ros::Publisher p_pub;
ros::Publisher cmdvel_pub;
ros::Publisher headingangle_pub;

// void setHeadingAngleCallback(const std_msgs::UInt8& val);

bool TrunService(zetabank_msgs::TurnSrv::Request & req, zetabank_msgs::TurnSrv::Response &res)
{
  ros::Rate loop_rate(50);

  ROS_INFO("Receive turn service...");

  if(b_RunTurnung == false)
  {
    b_RunTurnung = true;

    int goal_hangle = req.degree;
    int diff_hangle = 0;
    int prev_diffhang = 0;
    // int d_diffAng = 0;
    int hAngle = 0;
    double target_angular;

    ROS_INFO("Turn angle : %d", goal_hangle);   

    // prev_headingAngle = headingAngle;
    hAngle = (int)(headingAngle  * MATH_RAD2DEG);

    // adjK = 1.0;
    // if(hAngle > 180)
    //   adjK = hAngle/360.0;

    diff_hangle = 360 % (goal_hangle - hAngle);
    // target_angspeed = Kp * adjK *(diff_hangle * MATH_DEG2RAD );

    ROS_INFO("[1] hangle:%d dangle:%d", hAngle, diff_hangle);
    // ROS_INFO("[1] hangle:%d dangle:%d tangspeed:%.2lf", hAngle, diff_hangle, target_angspeed);

    geometry_msgs::Twist twist;

    while( abs(diff_hangle) >= 2)
    {
      hAngle = (int)(headingAngle  * MATH_RAD2DEG);

      diff_hangle = 360 % (goal_hangle - hAngle);
      // d_diffAng = prev_diffhang - diff_hangle;
      target_angular = Kp * (diff_hangle * MATH_DEG2RAD) + Kd * (prev_diffhang - diff_hangle);

      if(target_angular > max_angular)
        target_angular = max_angular;      
      if(target_angular < min_angular)
        target_angular = min_angular;

      twist.angular.z = target_angular;
      cmdvel_pub.publish(twist);

      ROS_INFO("[2] hangle:%d dangle:%d tangular:%.2lf", hAngle, diff_hangle, target_angular);

      loop_rate.sleep();
      // ros::spinOnce();

      prev_diffhang = diff_hangle;

    }

    res.result = true;

    b_RunTurnung = false;

    ROS_INFO("Complete turning process!");

    return true;
  }
}

// void setHeadingAngleCallback(const std_msgs::UInt8& val)
// {
//   if(bool_msg.data == true)
//     {
//       // ????
// #ifdef _DEBUG_BASIC        
//         ROS_INFO_STREAM("Heading angle is zero.");
// #endif    
//     }
// }

int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // configuring parameters
  std::string map_frame, base_frame;
  double publish_frequency;
  bool is_stamped;

  nh_priv.param<std::string>("map_frame",map_frame,"/map");
  nh_priv.param<std::string>("base_frame",base_frame,"/base_link");
  nh_priv.param<double>("publish_frequency",publish_frequency,10);
  nh_priv.param<bool>("is_stamped", is_stamped, false);

  if(is_stamped)
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
  else 
    p_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 10);


  // cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  headingangle_pub = nh.advertise<std_msgs::Int16>("heading_angle", 10);

  // ros::Subscriber bat_subscriber = node_obj.subscribe("set_heading_angle", 100, resetHeadingAngleCallback);

  // ros::ServiceServer turn_srv = nh.advertiseService("turn", TrunService);

  // ROS_INFO("Ready turn service...");

  // create the listener
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));

  ros::Rate rate(publish_frequency);

  tf::StampedTransform transform;
  // tf::Transform transf; 
  std_msgs::Int16 hang;

  ROS_INFO("Start robot_pose_publisher...");

  short get_angle = 0;
 

  while (nh.ok())
  {
    rate.sleep();
    ros::spinOnce();   

    try
    {
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);

      // construct a pose message
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();

      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();

      if(is_stamped)
        p_pub.publish(pose_stamped);
      else
      {
        p_pub.publish(pose_stamped.pose);
        // printf("robot_pose ==> publish psoe\n");
      }

      headingAngle = tf::getYaw(pose_stamped.pose.orientation);
      // headingAngle = transf.getYaw(pose_stamped.pose.orientation);

      get_angle = (short)(headingAngle * MATH_RAD2DEG);
      if(get_angle < 0)
        get_angle = get_angle + 359;

      // headingAngle = headingAngle * MATH_RAD2DEG;
      hang.data = get_angle;

      headingangle_pub.publish(hang);

      //ROS_INFO("hangle:%.2lf", hang.data);

    }
    catch (tf::TransformException &ex)
    {
      // just continue on
    }    

  }

  return EXIT_SUCCESS;
}
