/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

#define __DEBUG
//#define ___DEBUG

class TeleopJoy
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, actbtn_axes, stopbtn_button;
  double l_scale_, a_scale_;
  double l_maxvel, a_maxvel;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist pre_vel;
  //geometry_msgs::Twist cur_vel, pre_vel, last_published_;
  //boost::mutex publish_mutex_;
  //bool actbtn_pressed;
  bool zero_twist_published_;
  ros::Timer timer_;

};

TeleopJoy::TeleopJoy():
  ph_("~"),
  linear_(4),
  angular_(3),
  actbtn_axes(5),
  stopbtn_button(3),
  l_scale_(0.6),
  a_scale_(0.3),
  l_maxvel(1.5),
  a_maxvel(1.5)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_actbtn", actbtn_axes, actbtn_axes);
  ph_.param("button_stopbtn", stopbtn_button, stopbtn_button);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("max_angular", a_maxvel, a_maxvel);
  ph_.param("max_linear", l_maxvel, l_maxvel);

  //actbtn_pressed = false;
  zero_twist_published_ = false;

  pre_vel.angular.z = 0.0;
  pre_vel.linear.x = 0.0;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);

  //timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TeleopJoy::publish, this));
  //timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TeleopJoy::publish, this));
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist cur_vel;
  ros::Rate loop_rate(50);
  float lv = joy->axes[linear_];
  float av = joy->axes[angular_];

  if(fabs(lv)>fabs(av))
    av = 0.0;
  else 
    lv = 0.0;

  cur_vel.angular.x = 0.0;
  cur_vel.angular.y = 0.0;
  //cur_vel.angular.z = 0.0;
  //cur_vel.linear.x = 0.0;
  cur_vel.linear.y = 0.0;
  cur_vel.linear.z = 0.0;

  //geometry_msgs::Twist vel;
  /*vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  last_published_ = vel;*/

  cur_vel.angular.z = a_scale_*-1.0f*av;
  //cur_vel.angular.z = a_scale_*av;

  //cur_vel.angular.z = a_scale_*joy->axes[angular_];
  if((cur_vel.angular.z>0.0) && (cur_vel.angular.z>a_maxvel))
	  cur_vel.angular.z = a_maxvel;
  else if((cur_vel.angular.z<0.0) && (cur_vel.angular.z<-1.0*a_maxvel))
	  cur_vel.angular.z = -1.0*a_maxvel;

  cur_vel.linear.x = l_scale_*lv;
  //cur_vel.linear.x = l_scale_*joy->axes[linear_];

  if((cur_vel.linear.x>0.0) && (cur_vel.linear.x>l_maxvel))
	  cur_vel.linear.x = l_maxvel;
  else if((cur_vel.linear.x<0.0) && (cur_vel.linear.x<-1.0*l_maxvel))
	  cur_vel.linear.x = -1.0*l_maxvel;


  //cur_vel.angular.z = a_maxvel*a_scale_*joy->axes[angular_];
  //cur_vel.linear.x = l_maxvel*l_scale_*joy->axes[linear_];

  //last_published_ = cur_vel;

#if 0
  if(joy->axes[actbtn_axes] == -1) {
    actbtn_pressed = true;
#ifdef __DEBUG        
    ROS_INFO_STREAM("act btn pressed");
#endif        
  } else {
    actbtn_pressed = false;
#ifdef __DEBUG        
    ROS_INFO_STREAM("act btn released");
#endif       
  }
#endif

  if(joy->buttons[stopbtn_button] == 1) {
    cur_vel.angular.z = 0.0f;
    cur_vel.linear.x = 0.0f;
    pre_vel = cur_vel;
  }

//if(joy->axes[actbtn_axes] == -1)
  if (joy->axes[actbtn_axes] <= -1)
  //if ((joy->axes[actbtn_axes] <= -1) && ((pre_vel.angular.z!=cur_vel.angular.z) || (pre_vel.linear.x!=cur_vel.linear.x)))  
  {
    vel_pub_.publish(cur_vel);
    zero_twist_published_=false;
    pre_vel = cur_vel;
#ifdef _DEBUG 
    ROS_INFO_STREAM("publish >>");
#endif
  }
  else if((joy->axes[actbtn_axes]>-1) && (!zero_twist_published_))
  //else if(!actbtn_pressed && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
#ifdef __DEBUG 
    ROS_INFO_STREAM("zero publish >>");
#endif
  }  

#ifdef ___DEBUG        
    ROS_INFO_STREAM("RLT : " << joy->axes[actbtn_axes]);
    ROS_INFO_STREAM("Up/Down Btn : " << joy->axes[linear_]);
    ROS_INFO_STREAM("Left/Right Btn : " << joy->axes[angular_]);
    ROS_INFO_STREAM("X Btn : " << joy->buttons[stopbtn_button]);    // Stop    
#endif

#ifdef __DEBUG        
    ROS_INFO_STREAM("l-vel(" << cur_vel.linear.x << "), a-vel(" << cur_vel.angular.z << ")");
    //ROS_INFO_STREAM("angular vel : " << cur_vel.angular.z);
#endif  

  loop_rate.sleep();
}

void TeleopJoy::publish()
{
 #if 0
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (actbtn_pressed)
  //if (actbtn_pressed && ((pre_vel.angular.z!=cur_vel.angular.z) || (pre_vel.linear.x!=cur_vel.linear.x)))
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
    pre_vel = last_published_;
#ifdef _DEBUG 
    ROS_INFO_STREAM("publish >>");
#endif
  }
  else if(!actbtn_pressed && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
#ifdef _DEBUG 
    ROS_INFO_STREAM("zero publish >>");
#endif
  }

 #endif 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;

  ros::spin();
}
