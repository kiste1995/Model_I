#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::TransformStamped transformStamped2;
  geometry_msgs::TransformStamped transformStamped3;
  geometry_msgs::TransformStamped transformStamped4;

  //set a tf inheriting map.
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "submap1";
  transformStamped.transform.translation.x = 1.0;
  transformStamped.transform.translation.y = 2.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;

  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

      
  //set a tf inheriting map.
  transformStamped2.header.frame_id = "map";
  transformStamped2.child_frame_id = "submap2";
  transformStamped2.transform.translation.x = 1.0;
  transformStamped2.transform.translation.y = 0.0;
  transformStamped2.transform.translation.z = 0.0;

  transformStamped2.transform.rotation.x = q.x();
  transformStamped2.transform.rotation.y = q.y();
  transformStamped2.transform.rotation.z = q.z();
  transformStamped2.transform.rotation.w = q.w();
 
  //set a tf inheriting map.
  transformStamped3.header.frame_id = "map";
  transformStamped3.child_frame_id = "submap3";
  transformStamped3.transform.translation.x = 1.0;
  transformStamped3.transform.translation.y = -2.0;
  transformStamped3.transform.translation.z = 0.0;

  q.setRPY(0, 0, 0);
  transformStamped3.transform.rotation.x = q.x();
  transformStamped3.transform.rotation.y = q.y();
  transformStamped3.transform.rotation.z = q.z();
  transformStamped3.transform.rotation.w = q.w();
 
  //set a tf inheriting map.
  transformStamped4.header.frame_id = "map";
  transformStamped4.child_frame_id = "submap4";
  transformStamped4.transform.translation.x = 1.0;
  transformStamped4.transform.translation.y = 0.0;
  transformStamped4.transform.translation.z = 0.0;

  q.setRPY(0, 0, 0);
  transformStamped4.transform.rotation.x = q.x();
  transformStamped4.transform.rotation.y = q.y();
  transformStamped4.transform.rotation.z = q.z();
  transformStamped4.transform.rotation.w = q.w();



  ros::Rate rate(10.0);
  
  while (node.ok()){
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    tfb.sendTransform(transformStamped2);
    tfb.sendTransform(transformStamped3);
    tfb.sendTransform(transformStamped4);
    rate.sleep();
    printf("sending\n");
  }

};
