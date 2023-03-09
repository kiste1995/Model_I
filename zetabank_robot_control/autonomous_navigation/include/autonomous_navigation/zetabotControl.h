#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <autonomous_navigation/zetabot_navigation.h>
#include <zetabank_msgs/CmdInfo.h>
 
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>
#include <thread>

#include <geometry_msgs/Point32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/MapMetaData.h>

#include <geometry_msgs/Twist.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>

#define SLOW_MODE 0
#define NORMAL_MODE 1
#define FAST_MODE 2

#define CMD_STOP_PUB_CNT 200
using namespace std;

class ZetabotControl{
    public:
        ZetabotControl(void);
        ~ZetabotControl(void);

    public:
        ros::NodeHandle nh_;

		ros::Publisher teleopPublisher;
		ros::Subscriber mapSubscriber;
		ros::Subscriber velSubscriber;
        ros::ServiceServer cmd_srv;
        ros::ServiceClient local_cost_client;
        ros::ServiceClient dwa_plan_client;
        ros::ServiceClient clear_costmap_client;

    	ZetabotNavigation zetanav;
        
        zetabank_msgs::CmdInfo current_cmdInfo; //command information for sending to the top

        nav_msgs::MapMetaData g_map_data;   
        std::vector<double> map_origin; // origin of map, [x, y]
        float map_resolution;  

        // map TF
        tf2_ros::TransformBroadcaster tfb;
        tf2::Quaternion q;

        //move to goal
        geometry_msgs::TransformStamped calcPixel2Pos(float goal_x, float goal_y, float goal_theta);

        // stop
        actionlib_msgs::GoalID stop;


        // To clear costmap
        std_srvs::Empty empty_srv;

        // Speed mode
        std_msgs::String pre_speed;
        std_msgs::String current_speed;
        dynamic_reconfigure::Config dyn_cfg;
        dynamic_reconfigure::BoolParameter dyn_bool;
        dynamic_reconfigure::IntParameter dyn_int;
        dynamic_reconfigure::DoubleParameter dyn_db;
        dynamic_reconfigure::StrParameter dyn_str;
        dynamic_reconfigure::Reconfigure srv;
        
        // Teleoperation
        std_msgs::Bool teleop_msg;

        // Current velocity
        geometry_msgs::Twist current_vel;

        // Count the number of cmd_vel publish since openCR can stuck from many publish
        unsigned int cmd_pub_cnt;

        // Callback func
        bool cmdInfoCb(zetabank_msgs::CmdInfo::Request  &req,zetabank_msgs::CmdInfo::Response &res);
        void mapInfoCb(nav_msgs::MapMetaData );
        void getVelCb(geometry_msgs::Twist);

        void doAutoNav();   // command infomation을 기반으로 앞으로의 행동 판단.
        void moveToGoal(float x, float y, float theta);

        // Execute the function according to commands which the robot receive
        void execTeleop(float, float);
        void execStop();
        void execMoveTo(geometry_msgs::TransformStamped);


        // Decide naviagation parameter according to speed variable of communication
        void updataDynCfg( string str);

        void updataSlowMode();
        void updataSlowLocalMap();
        void updataSlowDWAPlan();

        void updataNormalMode();
        void updataNormalLocalMap();
        void updataNormalDWAPlan();

        void updataFastMode();
        void updataFastLocalMap();
        void updataFastDWAPlan();


};
