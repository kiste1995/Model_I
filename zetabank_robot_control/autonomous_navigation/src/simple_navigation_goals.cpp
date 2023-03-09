#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //receive tf data between map and submap  
  ros::NodeHandle node;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);    
  geometry_msgs::TransformStamped transformStamped;
  int cnt = 1;
      


  ros::Rate rate(10.0);

  while (node.ok())
  {  
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    try
    {
      if(cnt == 1)
      {
        ROS_INFO("transformStamp1");
        transformStamped = tfBuffer.lookupTransform("map", "submap1", ros::Time(0));    
        cnt++;
      }
      else if(cnt == 2)
      {
        ROS_INFO("transformStamp2");
        transformStamped = tfBuffer.lookupTransform("map", "submap2", ros::Time(0));   
        cnt++; 
         }
      else if(cnt == 3)
      {
        ROS_INFO("transformStamp3"); 
        transformStamped = tfBuffer.lookupTransform("map", "submap3", ros::Time(0));    
        cnt++;
      }
      else if(cnt == 4)
      {
        ROS_INFO("transformStamp4");
        transformStamped = tfBuffer.lookupTransform("map", "submap4", ros::Time(0));    
        cnt++;
        
        //Set the dynparam of dynamic_reconfigure.
        //system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.0");
     
      }
      else if(cnt == 5)
      {
        ROS_INFO("transformStamp4");
        transformStamped = tfBuffer.lookupTransform("map", "submap1", ros::Time(0));    
        cnt = 2;
      }
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      //continue;
    }
      


    //wait for the action server to come up
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = transformStamped.transform.translation.x;
    goal.target_pose.pose.position.y = transformStamped.transform.translation.y;
    goal.target_pose.pose.position.z = transformStamped.transform.translation.z;
    goal.target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
    goal.target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
    goal.target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
    goal.target_pose.pose.orientation.w = transformStamped.transform.rotation.w;


    ROS_INFO("Sending goal");
    ROS_INFO_STREAM("x : " << transformStamped.transform.translation.x << ", y : " <<transformStamped.transform.translation.y);
    
    try
    {
      ac.sendGoal(goal);

      ac.waitForResult();
    }
    catch(std::runtime_error& ex) 
    {
      ROS_ERROR("Exception: [%s]", ex.what());
    }

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, the base moved 1 meter forward");
    }
    else
    {
      ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
  }

  return 0;
}
