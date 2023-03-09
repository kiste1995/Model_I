#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <zetabank_msgs/Waypoint.h>
#include <zetabank_msgs/WaypointList.h>
#include <zetabank_msgs/Trajectory.h>
#include <zetabank_msgs/TrajectoryList.h>
#include <zetabank_msgs/NavigationControl.h>
#include <zetabank_msgs/NavigationControlStatus.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <zetabank_msgs/TurnSrv.h>
#include <zetabank_msgs/BreakTurn.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>

#include <stdio.h>
#include <cstdio>
#include <queue>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <ctime>
#include <unistd.h>
#include <string.h>

#include <boost/thread/mutex.hpp>
boost::mutex hangle_mutex;
boost::mutex sonar_mutex;

#define POINTNUM            50
#define PI                  3.1415926535897932384626433832795
#define MATH_RAD2DEG        57.2957795f
#define MATH_DEG2RAD        0.0174532f

template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class navigation_WayPoints
{
    public:
        navigation_WayPoints();
        ~navigation_WayPoints();
        bool init();
        bool loadWaypoints();    
        void naviCtrlCallback(const zetabank_msgs::NavigationControl::ConstPtr& navi_ctrl);
		void EStopCallback(const std_msgs::UInt8::ConstPtr& msg);
        void waypointNaviCtrl();

        void RunTurning(int yaw);
        void headingAngleCallback(const std_msgs::Int16& msg);
        void breakTurnCallback(const zetabank_msgs::BreakTurn& msg);
        void SonarCallback(const std_msgs::Float32MultiArray& msg);
        void SetSonarCallback(const std_msgs::Bool& bool_msg);

        int modAngle(int ang);
        bool b_wpfile_readOK;
        int target_turnangle;
        bool bBackward_Move;
        double th_reardist;
        double th_frontdist;

        double up_rear_sonar_dist[2];        
        double down_rear_sonar_dist[2];
        double down_left_sonar_dist[2];
        double down_right_sonar_dist[2];

    private:
    
        bool cancelGoal(double timeout = 2.0);
        void resetWaypoints();
        void publishStatusUpdate(const uint8_t& status);
        void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf);
        double distance2D(const tf::Transform& a, const tf::Transform& b = tf::Transform());

        void clear_costmap();

        std::string waypoints_fname;
        int wp_count;
        bool b_has_waypoint_fname;      
        YAML::Node doc;

        double _frequency;
        double _goal_timeout;
        double _close_enough;
        std::string _robot_frame;
        std::string _world_frame;
        std::string _wp_name;
		unsigned char bEStop_Status;

        tf::TransformListener _tf_listener;

        std::vector<geometry_msgs::PoseStamped> _waypoints;
        std::vector<geometry_msgs::PoseStamped>::iterator _waypoints_it;

        zetabank_msgs::WaypointList _wp_list;
        zetabank_msgs::TrajectoryList _traj_list;

        MoveBaseClient *move_base_ac;
        geometry_msgs::PoseStamped _goal;
        bool idle_status_update_sent;

        const geometry_msgs::PoseStamped NOWHERE;

        int headingAngle;
        bool bBreakTurn;
        double Kp;
        double Kd;
        double min_angular;
        double max_angular;

        ros::Publisher cmdvel_pub;
        ros::Subscriber hang_subsciber;
        ros::Subscriber breakturn_subsciber;

        bool bRunTurning;

        string abort_mode;
        double back_time;
        

        enum {  NONE = 0,
                GOAL,
                LOOP,
        } _mode;

        enum {  IDLE = 0,
                START,
                ACTIVE,
                COMPLETED,
				WARNING,
				ERROR,
				ESTOP,
                ABORTED,
        } _state, _prev_state;

        enum { CIDLE = 0,
               CSTART,
               CSTOP,
               CCANCELED
        } _control;
		
		enum { WP = 1,
			   TRAJ
		} _runtype;
			

        ros::Subscriber navi_ctrl_subsciber;
        ros::Subscriber estop_subsciber;
        ros::Subscriber sonar_subsciber;
        ros::Subscriber setsonar_subscriber;

        ros::Publisher status_pub;

         ros::ServiceClient clear_costmaps_client;

        bool bUseSonar;
        int aborted_cnt;
        int faborted_cnt;
};


navigation_WayPoints::navigation_WayPoints()
{
   _mode = NONE;
   _state = IDLE;
   _prev_state = IDLE;
   _runtype = WP;
   _frequency = 5;
   _goal_timeout = 300.0;
   _close_enough = 0.1;
    headingAngle = 0;
    bBreakTurn = false;

    Kp = 0.1;
    Kd = 0.2;

    min_angular = 0.1;
    max_angular = 1.2;
    target_turnangle = 0;

    bRunTurning = false;

    abort_mode = "stop";
    back_time = 2.0;

    bBackward_Move = false;

    bUseSonar = false;
    aborted_cnt = 0;
    faborted_cnt = 0;
}

navigation_WayPoints::~navigation_WayPoints()
{

}

bool navigation_WayPoints::init()
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    move_base_ac = new MoveBaseClient("move_base", true);

    hang_subsciber = nh.subscribe("heading_angle", 10, &navigation_WayPoints::headingAngleCallback, this);
    cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    breakturn_subsciber = nh.subscribe("break_turn", 10, &navigation_WayPoints::breakTurnCallback, this);

    status_pub = nh.advertise<zetabank_msgs::NavigationControlStatus>("navi_ctrl_status", 1, true);
    navi_ctrl_subsciber = nh.subscribe("navi_ctrl", 10, &navigation_WayPoints::naviCtrlCallback, this);   
    estop_subsciber = nh.subscribe("EmergencyStop", 2, &navigation_WayPoints::EStopCallback, this);

    sonar_subsciber = nh.subscribe("sonar", 20, &navigation_WayPoints::SonarCallback, this);
    setsonar_subscriber = nh.subscribe("set_sonar",100, &navigation_WayPoints::SetSonarCallback, this);

    clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    nh_private.param<std::string>("abort_mode", abort_mode, "stop");
    ROS_INFO("abort_mode : %s.", abort_mode.c_str());
    nh_private.param("back_time", back_time, 2.0);
    ROS_INFO("back_time : %lf", back_time);
    nh_private.param("th_reardist", th_reardist, 2.0);
    ROS_INFO("threshold rear distance : %lf", th_reardist);
    // nh_private.param("th_frontdist", th_frontdist, 2.0);
    // ROS_INFO("threshold front distance : %lf", th_frontdist);


    if(nh_private.getParam("waypoints_filename", waypoints_fname))
    {
        b_has_waypoint_fname = true;
        b_wpfile_readOK = false;
        wp_count = 0;

        ROS_INFO("File name is %s.", waypoints_fname.c_str());
	
        _waypoints.clear();
        _waypoints_it = _waypoints.end();
		
		bEStop_Status = 0;

        ROS_INFO("Navigation Mode : Goal");

        while((move_base_ac->waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
        {
            ROS_INFO("Waiting for the move_base action server to come up");         
            
        }
    }
    else 
    {
        b_has_waypoint_fname = false;
        ROS_WARN("No wayoints file name(%s) given as parameter.", waypoints_fname.c_str());        
    }
}

void navigation_WayPoints::EStopCallback(const std_msgs::UInt8::ConstPtr& msg)
{
	bEStop_Status = msg->data;
	
	if (bEStop_Status != 1)
	{
        if (bEStop_Status == 0)
            ROS_INFO_STREAM("Detected Lidar Field");
        else if (bEStop_Status == 3)
            ROS_INFO_STREAM("Pushed E-Stop Button");
        else if (bEStop_Status == 2)
            ROS_INFO_STREAM("Detected Lidar Field and Pushed E-Stop Button");

		_prev_state = _state;
		_state = ESTOP;
	}
    else if (bEStop_Status == 1 && _state == ESTOP)
	{
        ROS_INFO_STREAM("Release E-stop/ Lidar detecting prevstate: " << _prev_state << "state: " << _state);
		
		_state = IDLE;
		_prev_state = ESTOP;
	}

    // 22. 07. 15
    // if (bEStop_Status != 0)
	// {
    //     if (bEStop_Status == 1)
    //         ROS_INFO_STREAM("Detected Lidar Field");
    //     else if (bEStop_Status == 2)
    //         ROS_INFO_STREAM("Pushed E-Stop Button");
    //     else if (bEStop_Status == 3)
    //         ROS_INFO_STREAM("Detected Lidar Field and Pushed E-Stop Button");

	// 	_prev_state = _state;
	// 	_state = ESTOP;
	// }
    // else if (bEStop_Status == 0 && _state == ESTOP)
	// {
    //     ROS_INFO_STREAM("Release E-stop/ Lidar detecting prevstate: " << _prev_state << "state: " << _state);
		
	// 	_state = IDLE;
	// 	_prev_state = ESTOP;
	// }
}

void navigation_WayPoints::naviCtrlCallback(const zetabank_msgs::NavigationControl::ConstPtr& navi_ctrl)
{
    if(navi_ctrl->control == zetabank_msgs::NavigationControl::STOP)
    {
        if((_state == START) || (_state == ACTIVE))
        {
            ROS_INFO_STREAM("Stopping current execution...");
            cancelGoal();
            resetWaypoints();
            _state = IDLE;
            publishStatusUpdate(zetabank_msgs::NavigationControlStatus::CANCELLED);            
        } else if(_state == ABORTED)
        {
            ROS_INFO_STREAM("Reset aborted state...");
            cancelGoal();
            resetWaypoints();
            _state = IDLE;
            publishStatusUpdate(zetabank_msgs::NavigationControlStatus::IDLING);   
        }
    } else if( (navi_ctrl->control == zetabank_msgs::NavigationControl::START))
    {
        if( (_state == IDLE) || (_state == COMPLETED))
        {
            resetWaypoints();

            bool goal_found = false;

            ROS_INFO_STREAM("Goal name : " << navi_ctrl->goal_name);

            for (unsigned int wp = 0; wp < _wp_list.waypoints.size(); ++wp)
            {
                if(navi_ctrl->goal_name == _wp_list.waypoints[wp].name)
                {
                    geometry_msgs::PoseStamped pose;
                    pose.header = _wp_list.waypoints[wp].header;
                    pose.pose = _wp_list.waypoints[wp].pose;
                    // CAUTION below ==============================
                    // target_turnangle = _wp_list.waypoints[wp].yaw;
                    //==============================================
                    _waypoints.push_back(pose);
                    _waypoints_it = _waypoints.begin();
                    goal_found = true;
					
					_runtype = WP;

                    _wp_name = navi_ctrl->goal_name;

                    ROS_INFO_STREAM("Prepared to navigate to way point '" << navi_ctrl->goal_name << "'." );
                    continue;                
                }
            }

            if(!goal_found)
            {
                for(unsigned int traj = 0; traj < _traj_list.trajectories.size(); ++traj)
                {
                    if(navi_ctrl->goal_name == _traj_list.trajectories[traj].name)
                    {
                        for(unsigned int wp = 0; wp < _traj_list.trajectories[traj].waypoints.size(); ++wp)
                        {
                            geometry_msgs::PoseStamped pose;
                            pose.header = _traj_list.trajectories[traj].waypoints[wp].header;
                            pose.pose = _traj_list.trajectories[traj].waypoints[wp].pose;
                            // CAUTION below ==============================
                            // target_turnangle = _traj_list.trajectories[traj].waypoints[wp].yaw;
                            //==============================================
                            _waypoints.push_back(pose);
                        }
                        _waypoints_it = _waypoints.begin();
                        goal_found = true;
						
						_runtype = TRAJ;
						
                        ROS_INFO_STREAM("Prepared to navigate along the trajectory '" << navi_ctrl->goal_name << "'.");
                    }
                }
            }

            if(goal_found)
            {
                if(navi_ctrl->mode == zetabank_msgs::NavigationControl::NONE)
                    _mode = NONE;
                else if(navi_ctrl->mode == zetabank_msgs::NavigationControl::GOAL)
                    _mode = GOAL;
                else if(navi_ctrl->mode == zetabank_msgs::NavigationControl::LOOP)
                    _mode = LOOP;
                else
                    _mode = NONE;

                _state = START;
                // _mode = GOAL;
            }
            else {
                ROS_WARN_STREAM("Could not find provided way point or trajectory.");
            }
        }
        else {
            ROS_WARN_STREAM("Cannot start way point/trajectory execution, because navigator is currently active." <<
            " Please stop cujrrent activity first.");
        }
    }
    else 
    {
        ROS_WARN_STREAM("'Pause' not yet implemented.");
    }

}

bool navigation_WayPoints::loadWaypoints(void)
{
    _wp_list.waypoints.clear();
    _traj_list.trajectories.clear();

    try 
    {
        std::ifstream ifs(waypoints_fname);

        doc = YAML::Load(ifs);

        const YAML::Node& wp_node_temp1 = doc["waypoints"];

        const YAML::Node* wp_node1 = wp_node_temp1 ? &wp_node_temp1 : NULL;

        if(wp_node1 != NULL)
        {

            ROS_INFO_STREAM("[waypoints]");

            for(int i = 0; i < wp_node1->size(); i++)
            {
                zetabank_msgs::Waypoint wp;

                (*wp_node1)[i]["name"] >> wp.name;
                (*wp_node1)[i]["frame_id"] >> wp.header.frame_id;
                (*wp_node1)[i]["pose"]["position"]["x"] >> wp.pose.position.x;
                (*wp_node1)[i]["pose"]["position"]["y"] >> wp.pose.position.y;
                (*wp_node1)[i]["pose"]["position"]["z"] >> wp.pose.position.z;
                //======== CAUTION ===============================
                // (*wp_node1)[i]["pose"]["angle"]["yaw"] >> wp.yaw;
                //================================================

                (*wp_node1)[i]["pose"]["orientation"]["x"] >> wp.pose.orientation.x;
                (*wp_node1)[i]["pose"]["orientation"]["y"] >> wp.pose.orientation.y;
                (*wp_node1)[i]["pose"]["orientation"]["z"] >> wp.pose.orientation.z;
                (*wp_node1)[i]["pose"]["orientation"]["w"] >> wp.pose.orientation.w;

                // pos.orientation = tf::createQuaternionMsgFromYaw(wp.yaw*MATH_DEG2RAD);

                // wp.pose.orientation.x = pos.orientation.x;
                // wp.pose.orientation.y = pos.orientation.y;
                // wp.pose.orientation.z = pos.orientation.z;
                // wp.pose.orientation.w = pos.orientation.w;

                _wp_list.waypoints.push_back(wp);

                ROS_INFO_STREAM("name: " << wp.name);
                ROS_INFO_STREAM("frame_id: " << wp.header.frame_id);
                ROS_INFO_STREAM("position x(" << wp.pose.position.x << "), y(" << wp.pose.position.y << "), z(" << wp.pose.position.z << ")");
                ROS_INFO_STREAM("orientation x(" << wp.pose.orientation.x << "), y(" << wp.pose.orientation.y << "), z(" << wp.pose.orientation.z << "), w(" << wp.pose.orientation.w << ")");
                // ROS_INFO_STREAM("heading angle: " << wp.yaw);
                
            }
        }
        else 
        {
            ROS_WARN_STREAM("Couldn't find any waypoints in the provided yaml file.");
        }

#if 1
        const YAML::Node& wp_node_temp2 = doc["trajectories"];

        const YAML::Node* wp_node2 = wp_node_temp2 ? &wp_node_temp2 : NULL;

        if(wp_node2 != NULL)
        {

            ROS_INFO_STREAM("[trajectories]");

            for(int i = 0; i < wp_node2->size(); i++)
            {
                zetabank_msgs::Trajectory traj;

                bool all_waypoints_found = true;

                for(int j = 0; j < (*wp_node2)[i]["waypoints"].size(); ++j)
                {
                    bool wp_found = false;
                    std::string wp_name;
                    (*wp_node2)[i]["waypoints"][j] >> wp_name;

                    ROS_INFO_STREAM("wp name[" << j+1 << "]: " << wp_name);

                    for(int k = 0; k < _wp_list.waypoints.size(); ++k)
                    {
                        if(wp_name == _wp_list.waypoints[k].name)
                        {
                            traj.waypoints.push_back(_wp_list.waypoints[k]);
                            wp_found = true;
                            break;

                        }
                    }
                    if(!wp_found)
                    {
                        all_waypoints_found = false;
                        break;
                    }
                }
                if (all_waypoints_found)
                {
                    (*wp_node2)[i]["name"] >> traj.name;
                    _traj_list.trajectories.push_back(traj);

                }
            }
            ROS_INFO_STREAM("Parsed " << _traj_list.trajectories.size() << " trajectories.");
        }
        else 
        {
            
        }
#endif

    }
    catch(YAML::ParserException& e)
    {
        ROS_ERROR("[1]Parsing waypoints file failed: %s", e.what());
        return false;
    }
    catch(YAML::RepresentationException& e)
    {
        ROS_ERROR("[2]Parsing waypoints file failed: %s", e.what());
        return false;
    }
    catch(std::string& e)
    {
        ROS_ERROR("[3]Parsing waypoints file failed: %s", e.c_str());
        return false;
    }

    return true;
}

void navigation_WayPoints::waypointNaviCtrl()
{
    unsigned int stime;
    move_base_msgs::MoveBaseGoal wp_goal;

    ros::Rate rate(_frequency);

    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();

        if(_state == START)
        {
            if(_mode == LOOP)
            {
                if(_waypoints_it == _waypoints.end())
                {
                    _waypoints_it = _waypoints.begin();
                }
            }

            if(_waypoints_it < _waypoints.end())
            {
                clear_costmap();

                wp_goal.target_pose.header.stamp = ros::Time::now();
                wp_goal.target_pose.header.frame_id = _waypoints_it->header.frame_id;
                wp_goal.target_pose.pose = _waypoints_it->pose;
                ROS_INFO("New goal => %f, %f, %f", wp_goal.target_pose.pose.position.x,
                    wp_goal.target_pose.pose.position.y, tf::getYaw(wp_goal.target_pose.pose.orientation));

                move_base_ac->sendGoal(wp_goal);

                publishStatusUpdate(zetabank_msgs::NavigationControlStatus::RUNNING);

                _state = ACTIVE;

                _prev_state = ACTIVE;
            }
            else
            {
                ROS_ERROR_STREAM("Cannot start execution. Already at the last way point.");
                idle_status_update_sent = false;
                _prev_state = _state;
                _state = IDLE;
            }
        }
        else if(_state == ACTIVE)
        {
            actionlib::SimpleClientGoalState goal_state = move_base_ac->getState();

            if((goal_state == actionlib::SimpleClientGoalState::ACTIVE) || 
                (goal_state == actionlib::SimpleClientGoalState::PENDING) ||
                (goal_state == actionlib::SimpleClientGoalState::RECALLED) ||
                (goal_state == actionlib::SimpleClientGoalState::PREEMPTED))
            {
                


                if((ros::Time::now() - wp_goal.target_pose.header.stamp).toSec() >= _goal_timeout)
                {
                    ROS_WARN("Cannot reach goal after %f seconds : current state is %s", _goal_timeout, move_base_ac->getState().toString().c_str());

                    if(_waypoints_it < (_waypoints.end() - 1))
                    {
						if(_runtype == TRAJ)
						{
							ROS_INFO_STREAM("Requesting newxt way point.");
							_waypoints_it++;
                            _prev_state = _state;
							_state = START;
							
							publishStatusUpdate(zetabank_msgs::NavigationControlStatus::WARNNRGTO);
							idle_status_update_sent = true;
						} else {
							publishStatusUpdate(zetabank_msgs::NavigationControlStatus::WARNNRGTO);
                            _prev_state = _state;
							_state = WARNING;
						}
                    }
                    else
                    {
						if(_runtype == TRAJ)
						{
							ROS_INFO_STREAM("No more way points to go to.");
							_state = COMPLETED;
							
							publishStatusUpdate(zetabank_msgs::NavigationControlStatus::WARNNRGTO);
							idle_status_update_sent = true;
						} else {
							publishStatusUpdate(zetabank_msgs::NavigationControlStatus::WARNNRGTO);
                            _prev_state = _state;
							_state = WARNING;
						}
                    }
                }

                if(_waypoints_it < (_waypoints.end() - 1))
                {
                    tf::StampedTransform robot_gb, goal_gb;

                    try
                    {
                        _tf_listener.lookupTransform(_world_frame, _robot_frame, ros::Time(0), robot_gb);                        
                    }
                    catch(tf::TransformException& e)
                    {
                        ROS_WARN("Cannot get tf %s => %s : %s", _world_frame.c_str(), _robot_frame.c_str(), e.what());
                        continue;
                    }

                    pose2tf(wp_goal.target_pose, goal_gb);
                    double distance = distance2D(robot_gb, goal_gb);
                    if(distance < _close_enough)
                    {
                        _waypoints_it++;
                        _prev_state = _state;
                        _state = START;
                        ROS_INFO("Close enough to goal(%f <= %f m)", distance, _close_enough);
                        ROS_INFO_STREAM("Get next way point.");
                    }
                    else
                    {
                        // keep going until get close enough
                    }

                }
                else
                {
                    // keep going, since we approaching last way point.
                }
            }
            else // SUCCEEDED, REJECTED, ABORTED, LOST
            {
                if(goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Go to goal successfully complted: %f, %f, %f", wp_goal.target_pose.pose.position.x,
                        wp_goal.target_pose.pose.position.y, tf::getYaw(wp_goal.target_pose.pose.orientation));
                    
                    if(_waypoints_it < (_waypoints.end() - 1))
                    {
                        ROS_INFO_STREAM("Requesting next way point.");
                        _waypoints_it++;
                        _prev_state = _state;
                        _state = START;
											
                    }
                    else
                    {
                        ROS_INFO_STREAM("Reached final way point.");

                        if((_mode == LOOP) && (_runtype == TRAJ))
                        {
                            _waypoints_it = _waypoints.begin();

                            _prev_state = _state;
                            _state = START;

                            ROS_INFO_STREAM("Restart way point.");                            
                        }
                        else
                        {
                            _prev_state = _state;
                            _state = COMPLETED;

                            ROS_INFO("turning : %d", target_turnangle);

                            bRunTurning = true;

                            ros::Duration(0.5).sleep();

                            RunTurning(target_turnangle);

                            bRunTurning = false;
						
                            if(_runtype == WP)
                            {
                                publishStatusUpdate(zetabank_msgs::NavigationControlStatus::COMPLETED);
                                idle_status_update_sent = true;
                                ROS_INFO_STREAM("===> run type : way point");
                            }
                            else if(_runtype == TRAJ)
                            {
                                publishStatusUpdate(zetabank_msgs::NavigationControlStatus::TRAJCOMPLETED);
                                idle_status_update_sent = true;
                                ROS_INFO_STREAM("===> run type : trajectory");
                            }
                        }
                    }
                }
                else if(goal_state == actionlib::SimpleClientGoalState::ABORTED)
                {
                    _state = ABORTED;

                    ROS_ERROR(">>> Aborted status!!!");

                    publishStatusUpdate(zetabank_msgs::NavigationControlStatus::ABORTED);

                    _state = ABORTED;

                }
                else
                {
                    ROS_ERROR("Go to goal failed : %s.", move_base_ac->getState().toString().c_str());

                    if(_waypoints_it < (_waypoints.end() - 1))
                    {
                        ROS_INFO_STREAM("Requesting next way point.");
                        _waypoints_it++;
                        _prev_state = _state;
                        _state = START;						
                    }
                    else 
                    {
                        ROS_INFO_STREAM("No more way points to moving.");
                        _prev_state = _state;
                        _state = IDLE;                        
                    }

                    publishStatusUpdate(zetabank_msgs::NavigationControlStatus::ERRGTGF);
					idle_status_update_sent = true;
                }
            }
        }
		else if(_state == WARNING)
        {
            publishStatusUpdate(zetabank_msgs::NavigationControlStatus::WARNNRGTO);
            idle_status_update_sent = true;
        }
		else if(_state == ERROR)
        {
            publishStatusUpdate(zetabank_msgs::NavigationControlStatus::ERRGTGF);
            idle_status_update_sent = true;
        }
        else if( _state == ABORTED)
        {
            if(abort_mode.compare("back") == 0)
            {
                bBackward_Move = true;
                cancelGoal();

                ros::Duration(1.0).sleep();         

                if(bUseSonar == true)
                {
                    sonar_mutex.lock();
                    double drsd1 = down_rear_sonar_dist[0];
                    double drsd2 = down_rear_sonar_dist[1];
                    double ursd1 = up_rear_sonar_dist[0];
                    double ursd2 = up_rear_sonar_dist[1];                    
                    sonar_mutex.unlock();               

                    if((drsd1 > th_reardist) && (drsd2 > th_reardist) && (ursd1 > th_reardist) && (ursd2 < th_reardist))
                    //if((down_rear_sonar_dist[0] > th_reardist) && (down_rear_sonar_dist[1] > th_reardist) && (up_rear_sonar_dist > th_reardist))
                    {

                        geometry_msgs::Twist twist;  

                        twist.linear.x = -0.03;
                        twist.linear.y = 0.0;
                        twist.linear.z = 0.0;
                        twist.angular.x = 0.0;
                        twist.angular.y = 0.0;
                        twist.angular.z = 0.0;
                        cmdvel_pub.publish(twist);

                        ros::Duration(back_time).sleep();

                        twist.linear.x = 0.0;
                        cmdvel_pub.publish(twist);

                        ROS_WARN(">>> move backward....");

                        ros::Duration(0.5).sleep();                        

                        _state = START;

                        ROS_WARN(">>> restart....");
                    }
                    else {
                        ROS_WARN(">>> There is an obstacle behind the robot, making it impossible for the robot to move backwards.");

                        // sonar_mutex.lock();
                        ROS_INFO("dr1( %lf ), dr2( %lf ), ur1( %lf), ur2(%lf)", drsd1, drsd2, ursd1, ursd2);
                        // ROS_INFO_STREAM("dr1(" << down_rear_sonar_dist[0] << "), dr2(" << down_rear_sonar_dist[1] << "), ur(" << up_rear_sonar_dist << ")");
                        // sonar_mutex.unlock();

                        _state = ABORTED;

                        publishStatusUpdate(zetabank_msgs::NavigationControlStatus::REAROBSTACLE);
                        
                    }    
                }
                else
                {
                    geometry_msgs::Twist twist;  

                    twist.linear.x = -0.03;
                    twist.linear.y = 0.0;
                    twist.linear.z = 0.0;
                    twist.angular.x = 0.0;
                    twist.angular.y = 0.0;
                    twist.angular.z = 0.0;
                    cmdvel_pub.publish(twist);

                    ros::Duration(back_time).sleep();

                    twist.linear.x = 0.0;
                    cmdvel_pub.publish(twist);

                    ROS_WARN(">>> move backward....");

                    ros::Duration(0.5).sleep();                        

                    _state = START;

                    ROS_WARN(">>> restart....");
                }                  

                bBackward_Move = false;

                aborted_cnt += 1;

                if (aborted_cnt > 5)
                {
                    aborted_cnt = 0;

                    ROS_WARN(">>> The error repeats and resets the robot's odometry..");

                    // target_turnangle = 359;

                    for(int i=0; i<3; i++)
                    {
                        if ((headingAngle >= 0) && (headingAngle < 180))
                            target_turnangle = 180;
                        else if ((headingAngle >= 180) && (headingAngle > 359))
                            target_turnangle = 359;
                        
                        ROS_INFO("turning angle(%d), heading angle(%d)", target_turnangle, headingAngle);

                        bRunTurning = true;

                        ros::Duration(0.5).sleep();

                        RunTurning(target_turnangle);
                    
                    }

                    ros::Duration(0.5).sleep();

                    RunTurning(headingAngle);

                    bRunTurning = false;

                    ros::Duration(0.5).sleep();                        

                    _state = START;

                    ROS_WARN(">>> restart....");

                }

            } 
            else if(abort_mode.compare("retry") == 0)
            {
                cancelGoal();

                ros::Duration(2.0).sleep();

                _state = START;

                ROS_WARN(">>>>> restart....");
            }
        }
		else if(_state == ESTOP)
        {
			ROS_ERROR("Emergency Stop : release to this situation...");
		} else if(_prev_state == ESTOP)
        {
			ROS_INFO_STREAM(" Release E-Stop : Return previous running state.");

             _prev_state = IDLE;
		}
		
    }

}

bool navigation_WayPoints::cancelGoal(double timeout)
{
    actionlib::SimpleClientGoalState goal_state = move_base_ac->getState();
    if( (goal_state != actionlib::SimpleClientGoalState::ACTIVE) &&
        (goal_state != actionlib::SimpleClientGoalState::PENDING) &&
        (goal_state != actionlib::SimpleClientGoalState::RECALLED) &&
        (goal_state != actionlib::SimpleClientGoalState::PREEMPTED) )
    {
        ROS_WARN("Cannot cancel move base goal(state:%s).", goal_state.toString().c_str());

        publishStatusUpdate(zetabank_msgs::NavigationControlStatus::WARNING);

        return true;
    }

    ROS_INFO("Canceling move base goal with %s state!", goal_state.toString().c_str());
    move_base_ac->cancelAllGoals();
    if(move_base_ac->waitForResult(ros::Duration(timeout)) == false)
    {
        ROS_WARN("Cancel move base goal didn't finish after %.2f seconds(state:%s)", timeout, goal_state.toString().c_str());

        publishStatusUpdate(zetabank_msgs::NavigationControlStatus::ERROR);

        return false;
    }

    ROS_WARN("Cancel move base goal succeed. Current state is %s", goal_state.toString().c_str());
    
    publishStatusUpdate(zetabank_msgs::NavigationControlStatus::CANCELLED);

    return true;
}

void navigation_WayPoints::resetWaypoints(void)
{
    ROS_INFO("All reset : delete waypoints, goal and set state to IDLE");
    _waypoints.clear();
    _waypoints_it = _waypoints.end();

    _goal = NOWHERE;
    _mode = NONE;
}


void navigation_WayPoints::publishStatusUpdate(const uint8_t& status)
{
    zetabank_msgs::NavigationControlStatus ncs_msg;

    if(status == zetabank_msgs::NavigationControlStatus::IDLING)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::IDLING;
        ncs_msg.status_description = "Idling";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::RUNNING)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::RUNNING;
        ncs_msg.status_description = "Navigation to way point.";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::PAUSED)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::PAUSED;
        ncs_msg.status_description = "Navigation on hold.";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::COMPLETED)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::COMPLETED;
        ncs_msg.status_description = "Reached final destination.";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::CANCELLED)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::CANCELLED;
        ncs_msg.status_description = "Navigation cancelled.";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }
	else if(status == zetabank_msgs::NavigationControlStatus::WARNNRGTO)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::WARNNRGTO;
        ncs_msg.status_description = "Warning-timeout reach goal.";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }
	else if(status == zetabank_msgs::NavigationControlStatus::ERRGTGF)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::ERRGTGF;
        ncs_msg.status_description = "Error-fail going goal.";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::ERROR)    
    {
        ROS_ERROR_STREAM("Cannot publish unknown status updated!");
    }
    else if(status == zetabank_msgs::NavigationControlStatus::TRAJCOMPLETED)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::TRAJCOMPLETED;
        ncs_msg.status_description = "Trajectory final destination.";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::ABORTED)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::ABORTED;
        ncs_msg.status_description = "Aborted Error.";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::REAROBSTACLE)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::REAROBSTACLE;
        ncs_msg.status_description = "Exist obstacle in back.";
        ncs_msg.goal_name = _wp_name;
        status_pub.publish(ncs_msg);
    }

}

void navigation_WayPoints::pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf)
{
    tf.stamp_ = pose.header.stamp;
    tf.frame_id_ = pose.header.frame_id;

    tf.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    tf.setRotation(q);
}

double navigation_WayPoints::distance2D(const tf::Transform& a, const tf::Transform& b)
{
    return std::sqrt(std::pow(b.getOrigin().x() - a.getOrigin().x(), 2) + std::pow(b.getOrigin().y() - a.getOrigin().y(), 2));
}

void navigation_WayPoints::headingAngleCallback(const std_msgs::Int16& msg)
{
    if(bRunTurning)
    {
        hangle_mutex.lock();
        headingAngle = msg.data;
        hangle_mutex.unlock();
        ROS_INFO("headingAngle:%d", headingAngle);
    }
}

void navigation_WayPoints::breakTurnCallback(const zetabank_msgs::BreakTurn& msg)
{
  bBreakTurn = msg.breakturn;
}

int navigation_WayPoints::modAngle(int ang)
{
    int ret_ang;
    if(ang < 0)
    {
        ret_ang = (abs(ang) % 360)* -1;
    }
    else {
        ret_ang = ang % 360;
    }

    return ret_ang;
}

void navigation_WayPoints::RunTurning(int yaw)
{
    int diff_hangle = 0;
    int hAngle = 0;
    double target_angular;
    int prev_diffhang = 0;
    int sign = 1;
    int goal_hangle = yaw;

    ros::Time::init();

    ros::Rate loop_rate(50);

    geometry_msgs::Twist twist;    

    ROS_INFO("start turning....");

    hangle_mutex.lock();
    hAngle = headingAngle;
    hangle_mutex.unlock();

    hAngle = modAngle(hAngle);
    diff_hangle = goal_hangle - hAngle;

    ROS_INFO("[1] hangle:%d dangle:%d", hAngle, diff_hangle);

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

        hangle_mutex.lock();
        hAngle = headingAngle;
        hangle_mutex.unlock();

        hAngle = modAngle(hAngle);
        diff_hangle = goal_hangle - hAngle;
        if(diff_hangle < 0)
            sign = -1;
        else 
            sign = 1;

        target_angular = Kp * (diff_hangle * MATH_DEG2RAD) + Kd * ((prev_diffhang - diff_hangle) * MATH_DEG2RAD);
    
        if(fabs(target_angular) > max_angular)
            target_angular = max_angular*sign;      
        if(fabs(target_angular) < min_angular)
            target_angular = min_angular*sign;

        twist.angular.z = target_angular;
        cmdvel_pub.publish(twist);

        prev_diffhang = diff_hangle;

    }

    twist.angular.z = 0.0;
    cmdvel_pub.publish(twist);
    
    ROS_INFO("b_RunTurnung is false...");

}

void navigation_WayPoints::clear_costmap()
{
    std_srvs::Empty srv;
    clear_costmaps_client.call(srv);
}

void navigation_WayPoints::SetSonarCallback(const std_msgs::Bool& bool_msg)
{
    bUseSonar = bool_msg.data;
    ROS_INFO("Use Sonar :%d", bUseSonar);
}

void navigation_WayPoints::SonarCallback(const std_msgs::Float32MultiArray& msg)
{
    // 0 : dl2
    // 1 : dl1
    // 2 : df
    // 3 : dr1
    // 4 : dr2
    // 5 : db1
    // 6 : db2
    // 7 : ur
    // 8 : ub
    // 9 : ul

    // if(bUseSonar == true)
    // {
        sonar_mutex.lock();

        std_msgs::Float32MultiArray sensor_sonar = msg;

        down_left_sonar_dist[0] = sensor_sonar.data[5];
        down_left_sonar_dist[1] = sensor_sonar.data[4];

        down_right_sonar_dist[0] = sensor_sonar.data[0];
        down_right_sonar_dist[1] = sensor_sonar.data[1];

        down_rear_sonar_dist[0] = sensor_sonar.data[2];
        down_rear_sonar_dist[1] = sensor_sonar.data[3];

        up_rear_sonar_dist[0] = sensor_sonar.data[6];
        up_rear_sonar_dist[1] = sensor_sonar.data[7];        

        sonar_mutex.unlock();

    // }
}

int main(int argc, char** argv)
{
    int rate_wc = 100;

    ros::init(argc, argv, "Navigation way point node");

    ROS_INFO("Run navigation way point.");

    navigation_WayPoints naviWP;

    naviWP.init();

    naviWP.loadWaypoints();

    naviWP.waypointNaviCtrl();

    return 0;

}
