#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
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

#include <stdio.h>
#include <cstdio>
#include <queue>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <ctime>

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

//zetabank_msgs::WaypointList wps;
//zetabank_msgs::TrajectoryList trajs;



class navigation_WayPoints
{
    public:
        //navigation_WayPoints();
        navigation_WayPoints();
        ~navigation_WayPoints();
        bool init();
        bool loadWaypoints();    
        void naviCtrlCallback(const zetabank_msgs::NavigationControl::ConstPtr& navi_ctrl);
        void waypointNaviCtrl();

        bool b_wpfile_readOK;

    private:
    
        bool cancelGoal(double timeout = 2.0);
        void resetWaypoints();
        void publishStatusUpdate(const uint8_t& status);
        void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf);
        double distance2D(const tf::Transform& a, const tf::Transform& b = tf::Transform());

        std::string waypoints_fname;
        //ros::NodeHandle _nh;        
        int wp_count;
        bool b_has_waypoint_fname;      
        YAML::Node doc;

        double _frequency;
        double _goal_timeout;
        double _close_enough;
        std::string _robot_frame;
        std::string _world_frame;

        tf::TransformListener _tf_listener;


        std::vector<geometry_msgs::PoseStamped> _waypoints;
        std::vector<geometry_msgs::PoseStamped>::iterator _waypoints_it;

        zetabank_msgs::WaypointList _wp_list;
        zetabank_msgs::TrajectoryList _traj_list;

        MoveBaseClient *move_base_ac;
        //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac;
        geometry_msgs::PoseStamped _goal;
        bool idle_status_update_sent;

        const geometry_msgs::PoseStamped NOWHERE;

        enum {  NONE = 0,
                GOAL,
                LOOP
        } _mode;

        enum {  IDLE = 0,
                START,
                ACTIVE,
                COMPLETED,
				WARNING,
				ERROR
        } _state;

        enum { CIDLE = 0,
               CSTART,
               CSTOP,
               CCANCELED
        } _control;
		
		enum { WP = 1,
			   TRAJ
		} _runtype;
			

        ros::Subscriber navi_ctrl_subsciber;

        ros::Publisher status_pub;
};

/*navigation_WayPoints::navigation_WayPoints() : _mode(NONE), _state(IDLE)
{
   
}*/

navigation_WayPoints::navigation_WayPoints()
{
   _mode = NONE;
   _state = IDLE;
   _runtype = WP;
   _frequency = 5;
   _goal_timeout = 120.0;
   _close_enough = 0.1;
}

navigation_WayPoints::~navigation_WayPoints()
{

}

bool navigation_WayPoints::init()
{
 //ros::NodeHandle node_obj;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    move_base_ac = new MoveBaseClient("move_base", true);

    if(nh_private.getParam("waypoints_filename", waypoints_fname))
    {
        b_has_waypoint_fname = true;
        b_wpfile_readOK = false;
        wp_count = 0;

        ROS_INFO("File name is %s.", waypoints_fname.c_str());

        //readWaypoint();

        status_pub = nh.advertise<zetabank_msgs::NavigationControlStatus>("navi_ctrl_status", 1, true);

        navi_ctrl_subsciber = nh.subscribe("navi_ctrl", 2, &navigation_WayPoints::naviCtrlCallback, this);

        _waypoints.clear();
        _waypoints_it = _waypoints.end();


        ROS_INFO("Navigation Mode : Goal");

        //move_base_msgs::MoveBaseGoal goal;

        //MoveBaseClient ac("move_base", true);

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
                    _waypoints.push_back(pose);
                    _waypoints_it = _waypoints.begin();
                    goal_found = true;
					
					_runtype = WP;

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
                            _waypoints.push_back(pose);
                        }
                        _waypoints_it = _waypoints.begin();
                        goal_found = true;
						
						_runtype = TRAJ;
						
                        ROS_INFO_STREAM("Prepared to navogate along the trajectory '" << navi_ctrl->goal_name << "'.");
                    }
                }
            }

            if(goal_found)
            {
                _state = START;
                _mode = GOAL;
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

    //std::string recv_str = str_msg.data.c_str();
    //ROS_INFO_STREAM("[NC] Receive msgs : " << recv_str);

#if 0
    if(recv_str.compare("Goal") == 0)
    {
        ROS_INFO("Navigation Mode : Goal");

         move_base_msgs::MoveBaseGoal goal;

        MoveBaseClient ac("move_base", true);

        while(!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");         
        }

        while(!wps.waypoints.empty())
        {

        }

    }
    else if(recv_str.compare("Loop") == 0)
    {
        ROS_INFO("Navigation Mode : Loop");
    }
#endif

}

bool navigation_WayPoints::loadWaypoints(void)
{
    _wp_list.waypoints.clear();
    _traj_list.trajectories.clear();

    try 
    {
        std::ifstream ifs(waypoints_fname);

        //doc = YAML::LoadFile(waypoints_fname.c_str());
        doc = YAML::Load(ifs);
        //YAML::Parser parser(ifs);
        //parser.GetNextDocument(doc);

        //ROS_INFO_STREAM("waypoint file text : " << doc);

        const YAML::Node& wp_node_temp1 = doc["waypoints"];

        //ROS_INFO_STREAM("wp_node_temp1 : " << wp_node_temp1);

        const YAML::Node* wp_node1 = wp_node_temp1 ? &wp_node_temp1 : NULL;

        if(wp_node1 != NULL)
        {
            //ROS_INFO_STREAM("wp_node_temp1 : " << *wp_node1);
            //ROS_INFO_STREAM("wp node size : " << wp_node1->size());

            ROS_INFO_STREAM("[waypoints]");

            for(int i = 0; i < wp_node1->size(); i++)
            {
                zetabank_msgs::Waypoint wp;

                //ROS_INFO_STREAM("[1]wp parse count : " << i+1);

                (*wp_node1)[i]["name"] >> wp.name;

                //ROS_INFO_STREAM("[2]wp parse name : " << wp.name);

                (*wp_node1)[i]["frame_id"] >> wp.header.frame_id;
                //ROS_INFO_STREAM("[3]wp parse frame_id : " << wp.header.frame_id);

                //ROS_INFO_STREAM("[4]wp parse position : " << (*wp_node1)[i]["pose"]["position"]);
                //ROS_INFO_STREAM("[5]wp parse position -> x: " << (*wp_node1)[i]["pose"]["position"]["x"]);

                (*wp_node1)[i]["pose"]["position"]["x"] >> wp.pose.position.x;
                //ROS_INFO_STREAM("[4]wp parse count : " << i+1);
                //ROS_INFO_STREAM("[4]wp parse position -> x: " << wp.pose.position.x);

                (*wp_node1)[i]["pose"]["position"]["y"] >> wp.pose.position.y;
                //ROS_INFO_STREAM("[5]wp parse position -> y: " << wp.pose.position.y);

                (*wp_node1)[i]["pose"]["position"]["z"] >> wp.pose.position.z;
                //ROS_INFO_STREAM("[6]wp parse position -> z: " << wp.pose.position.z);

                (*wp_node1)[i]["pose"]["orientation"]["x"] >> wp.pose.orientation.x;
                //ROS_INFO_STREAM("[7]wp parse orientation -> x: " << wp.pose.orientation.x);

                (*wp_node1)[i]["pose"]["orientation"]["y"] >> wp.pose.orientation.y;
                //ROS_INFO_STREAM("[8]wp parse orientation -> y: " << wp.pose.orientation.y);

                (*wp_node1)[i]["pose"]["orientation"]["z"] >> wp.pose.orientation.z;
                //ROS_INFO_STREAM("[9]wp parse orientation -> z: " << wp.pose.orientation.z);

                (*wp_node1)[i]["pose"]["orientation"]["w"] >> wp.pose.orientation.w;
                //ROS_INFO_STREAM("[10]wp parse orientation -> w: " << wp.pose.orientation.w);               

                _wp_list.waypoints.push_back(wp);

                ROS_INFO_STREAM("name: " << wp.name);
                ROS_INFO_STREAM("frame_id: " << wp.header.frame_id);
                ROS_INFO_STREAM("position x(" << wp.pose.position.x << "), y(" << wp.pose.position.y << "), z(" << wp.pose.position.z << ")");
                ROS_INFO_STREAM("orientation x(" << wp.pose.orientation.x << "), y(" << wp.pose.orientation.y << "), z(" << wp.pose.orientation.z << "), w(" << wp.pose.orientation.w << ")");
                
            }
        }
        else 
        {
            ROS_WARN_STREAM("Couldn't find any waypoints in the provided yaml file.");
        }

#if 1
        const YAML::Node& wp_node_temp2 = doc["trajectories"];
        //ROS_INFO_STREAM("wp_node_temp2 : " << wp_node_temp2);

        const YAML::Node* wp_node2 = wp_node_temp2 ? &wp_node_temp2 : NULL;

        if(wp_node2 != NULL)
        {

            ROS_INFO_STREAM("[trajectories]");

           //ROS_INFO_STREAM("wp_node2 : " << *wp_node2);
           // ROS_INFO_STREAM("wp node2 size : " << wp_node2->size());

            for(int i = 0; i < wp_node2->size(); i++)
            {
                zetabank_msgs::Trajectory traj;

                bool all_waypoints_found = true;

                for(int j = 0; j < (*wp_node2)[i]["waypoints"].size(); ++j)
                {
                    //ROS_INFO_STREAM("[1]traj parse count : " << j+1);

                    bool wp_found = false;
                    std::string wp_name;
                    (*wp_node2)[i]["waypoints"][j] >> wp_name;

                    ROS_INFO_STREAM("wp name[" << j+1 << "]: " << wp_name);

                    //ROS_INFO_STREAM("[2]traj parse name : " << wp_name);

                    for(int k = 0; k < _wp_list.waypoints.size(); ++k)
                    {
                        if(wp_name == _wp_list.waypoints[k].name)
                        {
                            traj.waypoints.push_back(_wp_list.waypoints[k]);
                            //ROS_INFO_STREAM("traj waypoint [" << k+1 << "]: " << wps.waypoints[k]);
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
                wp_goal.target_pose.header.stamp = ros::Time::now();
                wp_goal.target_pose.header.frame_id = _waypoints_it->header.frame_id;
                wp_goal.target_pose.pose = _waypoints_it->pose;
                ROS_INFO("New goal => %f, %f, %f", wp_goal.target_pose.pose.position.x,
                    wp_goal.target_pose.pose.position.y, tf::getYaw(wp_goal.target_pose.pose.orientation));

                move_base_ac->sendGoal(wp_goal);

                publishStatusUpdate(zetabank_msgs::NavigationControlStatus::RUNNING);

                _state = ACTIVE;
            }
            else
            {
                ROS_ERROR_STREAM("Cannot start execution. Already at the last way point.");
                idle_status_update_sent = false;
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
							_state = START;
							
							publishStatusUpdate(zetabank_msgs::NavigationControlStatus::WARNNRGTO);
							idle_status_update_sent = true;
						} else {
							publishStatusUpdate(zetabank_msgs::NavigationControlStatus::WARNNRGTO);
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
                        _state = START;
						
						if(_runtype == TRAJ)
						{
							publishStatusUpdate(zetabank_msgs::NavigationControlStatus::COMPLETED);
							idle_status_update_sent = true;
						}
                    }
                    else
                    {
                        ROS_INFO_STREAM("Reached final way point.");
                        _state = COMPLETED;
						
						if(_runtype == WP)
						{
							publishStatusUpdate(zetabank_msgs::NavigationControlStatus::COMPLETED);
							idle_status_update_sent = true;
						}
						else if(_runtype == TRAJ)
						{
							publishStatusUpdate(zetabank_msgs::NavigationControlStatus::IDLING);
							idle_status_update_sent = true;
						}
                    }
                }
                else
                {
                    ROS_ERROR("Go to goal failed : %s.", move_base_ac->getState().toString().c_str());

                    if(_waypoints_it < (_waypoints.end() - 1))
                    {
                        ROS_INFO_STREAM("Requesting next way point.");
                        _waypoints_it++;
                        _state = START;
						
                    }
                    else 
                    {
                        ROS_INFO_STREAM("No more way points to moving.");
                        _state = IDLE;
                    }

                    publishStatusUpdate(zetabank_msgs::NavigationControlStatus::ERRGTGF);
					idle_status_update_sent = true;
                }
            }
        }
        /*else if(_state == COMPLETED)
        {
            publishStatusUpdate(zetabank_msgs::NavigationControlStatus::IDLING);
            idle_status_update_sent = true;
        } */
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

        publishStatusUpdate(zetabank_msgs::NavigationControlStatus::ERROR);

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
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::RUNNING)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::RUNNING;
        ncs_msg.status_description = "Navigation to way point.";
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::PAUSED)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::PAUSED;
        ncs_msg.status_description = "Navigation on hold.";
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::COMPLETED)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::COMPLETED;
        ncs_msg.status_description = "Reached final destination.";
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::CANCELLED)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::CANCELLED;
        ncs_msg.status_description = "Navigation cancelled.";
        status_pub.publish(ncs_msg);
    }
	else if(status == zetabank_msgs::NavigationControlStatus::WARNNRGTO)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::WARNNRGTO;
        ncs_msg.status_description = "Warning-timeout reach goal.";
        status_pub.publish(ncs_msg);
    }
	else if(status == zetabank_msgs::NavigationControlStatus::ERRGTGF)
    {
        ncs_msg.status = zetabank_msgs::NavigationControlStatus::ERRGTGF;
        ncs_msg.status_description = "Error-fail going goal.";
        status_pub.publish(ncs_msg);
    }
    else if(status == zetabank_msgs::NavigationControlStatus::ERROR)    
    {
        ROS_ERROR_STREAM("Cannot publish unknown status updated!");
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

int main(int argc, char** argv)
{
    int rate_wc = 100;
    //zetabank_msgs::WaypointList wps;
    //zetabank_msgs::TrajectoryList trajs;

    ros::init(argc, argv, "Navigation way point node");

    ROS_INFO("Run navigation way point.");

    //ros::NodeHandle node_obj("~");

    navigation_WayPoints naviWP;
    //navigation_WayPoints naviWP(node_obj);

    naviWP.init();

    naviWP.loadWaypoints();

    naviWP.waypointNaviCtrl();

    return 0;

}
