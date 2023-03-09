
#ifndef MOVEMENT_STATUS_H_
#define MOVEMENT_STATUS_H_

#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <actionlib/client/simple_action_client.h>

#include <map>
#include <string>
#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <cmath>

#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>

#include <zetabank_msgs/CommInfo.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>

using namespace std;

class ZetabotStatus{
    public:
        ZetabotStatus(void);
        ~ZetabotStatus(void);

    public:
        ros::NodeHandle nh_;
        ros::Publisher totalStatus_pub;
        ros::Publisher sonar_br_pub;
        ros::Publisher sonar_bl_pub;
        ros::Subscriber battery_sub;
        ros::Subscriber bumper_sub;
        ros::Subscriber sonar_sub;
        ros::Subscriber led_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber vel_sub;
        ros::Subscriber resp_sub;
        
		ros::Subscriber mapSubscriber;
        nav_msgs::MapMetaData g_map_data;   
        
        zetabank_msgs::CommInfo comminfo_msg;

        geometry_msgs::Point current_pt;

        sensor_msgs::Range sonar_br_msg;
        sensor_msgs::Range sonar_bl_msg;
        std_msgs::String total_sonar_msg;

        void publishZetabotStatus();        
        void sonarSplit(const char* total_sonar);

        void batteryCb(const sensor_msgs::BatteryStateConstPtr& str);
        void bumperCb(const std_msgs::StringConstPtr& str);
        void sonarCb(const std_msgs::StringConstPtr& str);
        void ledCb(const std_msgs::StringConstPtr& str);
        void lidarCb(const std_msgs::StringConstPtr& str);
        void poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
        void velCb(const geometry_msgs::TwistConstPtr& vel);
        void respCb(const std_msgs::StringConstPtr& str);

        void mapInfoCb(nav_msgs::MapMetaData);
        geometry_msgs::Point calcPosToPixel(float, float);

        unsigned int sonar_br_cnt;
        unsigned int sonar_bl_cnt;
        unsigned int total_sonar_br_cnt;
        unsigned int total_sonar_bl_cnt;


};

#endif // MOVEMENT_STATUS_H_