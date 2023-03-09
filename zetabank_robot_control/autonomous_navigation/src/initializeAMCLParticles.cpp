#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include <sstream>

#define Threshold_distance 5

int seq_count;
float threshold_covariance;
geometry_msgs::PoseWithCovarianceStamped gMsg;
geometry_msgs::PoseStamped currentGoal;

ros::Publisher pub;

std::array<float,36> init_cov_arr = {
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
    }; 

void setCurrentGoal(geometry_msgs::PoseStamped goal) 
{
	currentGoal = goal;
}        

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    gMsg.header.seq = msg->header.seq + 1;
    gMsg.header.frame_id = "map";
    
    // Error: tf: Lookup would require extrapolation into the past
    // https://answers.ros.org/question/188023/tf-lookup-would-require-extrapolation-into-the-past/
    gMsg.header.stamp = ros::Time(0);
    gMsg.pose = msg->pose;
    
    threshold_covariance = msg->pose.covariance[0];

    ROS_INFO_STREAM("Get AMCL Pose");

    // Consider covariance and difference distance between amcl_pose and goal_pose, which second condition can occurs repetable action. 
    if(threshold_covariance < 0.005f  && 
    (abs(currentGoal.pose.position.x - gMsg.pose.pose.position.x)+ abs(currentGoal.pose.position.y - gMsg.pose.pose.position.y))  > Threshold_distance)
    {
        ROS_INFO_STREAM("Spread the particles");

        for(int i = 0; i<36; i++)
        {
            gMsg.pose.covariance[i] = init_cov_arr[i];
        }
        
        pub.publish(gMsg);

    }
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "init_Partcles");

  ros::NodeHandle n;

  pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
  ros::Subscriber amcl_pose_sub = n.subscribe("/amcl_pose", 1000, poseCallback);
  ros::Subscriber goal_pose_sub = n.subscribe("/move_base/current_goal", 1000, setCurrentGoal);

  seq_count = 0;


  ros::spin();

  return 0;
}
